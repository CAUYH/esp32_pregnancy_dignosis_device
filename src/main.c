/**
 * @brief 母兔孕征诊断设备
 *
 *
 */

#include "easyio.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"
#include "ble_spp_server_demo.h"

// GPIO
#define LED 2
#define BOOT 0
#define KEY_USR 15
#define PRESS_SENSOR 36
#define LONG_WAVE_LED 12
#define SHORT_WAVE_LED 13
#define RED_LED 14
// USER_DEFINE
#define ADC_DUPLICATE_READ_NUM 64

static const char *TAG = "main";
//光电二极管采样值
struct PD_VALUE
{
    uint32_t re_pd_1;
    uint32_t re_pd_2;
    uint32_t sw_pd_1;
    uint32_t sw_pd_2;
    uint32_t lw_pd_1;
    uint32_t lw_pd_2;
};
struct PD_VALUE pd_value = {
    0, 0, 0, 0};
//蜂鸣器特征
struct BEEP
{
    uint8_t beep_status; //蜂鸣器鸣响状态，1：开启 0：关闭
    uint8_t beep_num;    //蜂鸣器连鸣次数
    uint16_t beep_delay; //蜂鸣器连鸣间隔，ms
};
struct BEEP beep =
    {
        0, 1, 200};
//屏幕特征
struct LCD
{
    uint8_t lcd_refresh_status; // LCD刷新状态
    char lcd_title_str[30];     //屏幕标题
    /* data */
};
struct LCD lcd = {
    1,
    "LCD TITLE ZONE",
};
//母兔特征
struct RABBIT
{
    /* data */
    uint16_t rabbit_number;      //母兔编号
    uint8_t preg_status;         //怀孕状态 1:怀孕 0:未怀孕
    uint8_t preg_day;            //怀孕天数
    uint8_t preg_produce_status; //母兔妊娠周期 1：生产期 0:后备期
    uint8_t cut_hair_status;     //母兔剃毛状态 0:未剃毛 1：剃毛
};
struct RABBIT rabbit = {
    0,
    1,
    13,
    1,
    0,
};
//按键特征
struct KEY
{
    /* data */
    uint8_t key_choose; //按键选择
    uint8_t key_num;    //功能按键数量
};
struct KEY key = {
    0,
    5,
};
//保存数据
struct DATA_SAVE
{
    /* data */
    uint8_t data_sampling_status; //数据采集完成标志
    uint8_t data_save_status;     //数据保存标志
    uint8_t data_save_response;   //数据保存相应标志 1：作出响应 0：未做响应
};
struct DATA_SAVE data_save = {
    0,
    0,
    0,
};

struct WORK_MODE
{
    /* data */
    uint8_t mode_choose;  //工作模式选择 0：测试模式 连续采样 并通过蓝牙发送至电脑显示波形
                          // 1:采样模式 每个通道采集一个数据 并保存
                          // 2:预测模式 每个通道采集一个数据 并预测
    char display_tag[10]; //工作模式的显示标签
    uint8_t probe_choose; //探头选择
    uint8_t test_flag;    //测试模式开启标签
    uint8_t test_fre;     //测试模式采样频率
};

struct WORK_MODE work_mode = {
    2,
    "test",
    1,
    1,
    10,
};

//不同命令发送缓冲区
struct BLE_COMMEND_BUFF
{
    /* data */
    uint8_t device_status[6]; //设备状态
};
struct BLE_COMMEND_BUFF ble_commend_buff = {
    //帧头 长度 地址 命令 数据 帧尾
    //地址和数据根据不同的状态进行更改
    {0x23, 0x06, 0x00, 0x0c, 0x00, 0x2a},
};

#define GATTS_TABLE_TAG "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#define SAMPLE_DEVICE_NAME "ESP_SPP_SERVER" // The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID 0

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE 0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY 0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE 0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY 0xABF4

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT 0xABF5
#endif

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02, 0x01, 0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03, 0x03, 0xF0, 0xAB,
    /* Complete Local Name in advertising */
    0x0F, 0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R', 'V', 'E', 'R'};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static xQueueHandle cmd_cmd_queue = NULL;

#ifdef SUPPORT_HEARTBEAT
static xQueueHandle cmd_heartbeat_queue = NULL;
static uint8_t heartbeat_s[9] = {'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
static bool enable_heart_ntf = false;
static uint8_t heartbeat_count_num = 0;
#endif

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {
    0x0,
};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node
{
    int32_t len;
    uint8_t *node_buff;
    struct spp_receive_data_node *next_node;
} spp_receive_data_node_t;

static spp_receive_data_node_t *temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t *temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff
{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t *first_node;
} spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num = 0,
    .buff_size = 0,
    .first_node = NULL};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

/// SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t spp_data_receive_val[20] = {0x00};

/// SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t spp_data_notify_val[20] = {0x00};
static const uint8_t spp_data_notify_ccc[2] = {0x00, 0x00};

/// SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t spp_command_val[10] = {0x00};

/// SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t spp_status_val[10] = {0x00};
static const uint8_t spp_status_ccc[2] = {0x00, 0x00};

#ifdef SUPPORT_HEARTBEAT
/// SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
    {
        // SPP -  Service Declaration
        [SPP_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

        // SPP -  data receive characteristic Declaration
        [SPP_IDX_SPP_DATA_RECV_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // SPP -  data receive characteristic Value
        [SPP_IDX_SPP_DATA_RECV_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_DATA_MAX_LEN, sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

        // SPP -  data notify characteristic Declaration
        [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // SPP -  data notify characteristic Value
        [SPP_IDX_SPP_DATA_NTY_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ, SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

        // SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_DATA_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

        // SPP -  command characteristic Declaration
        [SPP_IDX_SPP_COMMAND_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // SPP -  command characteristic Value
        [SPP_IDX_SPP_COMMAND_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_CMD_MAX_LEN, sizeof(spp_command_val), (uint8_t *)spp_command_val}},

        // SPP -  status characteristic Declaration
        [SPP_IDX_SPP_STATUS_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // SPP -  status characteristic Value
        [SPP_IDX_SPP_STATUS_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ, SPP_STATUS_MAX_LEN, sizeof(spp_status_val), (uint8_t *)spp_status_val}},

        // SPP -  status characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_STATUS_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

#ifdef SUPPORT_HEARTBEAT
        // SPP -  Heart beat characteristic Declaration
        [SPP_IDX_SPP_HEARTBEAT_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

        // SPP -  Heart beat characteristic Value
        [SPP_IDX_SPP_HEARTBEAT_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

        // SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_HEARTBEAT_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB; i++)
    {
        if (handle == spp_handle_table[i])
        {
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if (temp_spp_recv_data_node_p1 == NULL)
    {
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if (temp_spp_recv_data_node_p2 != NULL)
    {
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
    if (SppRecvDataBuff.node_num == 0)
    {
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }
    else
    {
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}
void spp_cmd_task(void *arg)
{
    uint8_t *cmd_id;

    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY))
        {
            esp_log_buffer_char(GATTS_TABLE_TAG, (char *)(cmd_id), strlen((char *)cmd_id));
            free(cmd_id);
        }
    }
    vTaskDelete(NULL);
}

static void spp_task_init(void)
{
    // spp_uart_init(); //串口初始化

    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 10, NULL); //创建spp_cmd任务
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: //连接上蓝牙
        esp_ble_gap_start_advertising(&spp_adv_params);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        sprintf(lcd.lcd_title_str, "ble is connect");
        lcd.lcd_refresh_status = 1; //刷新LCD
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: //断开蓝牙
        // advertising start complete event to indicate advertising start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        sprintf(lcd.lcd_title_str, "ble is disconnect");
        lcd.lcd_refresh_status = 1; //刷新LCD
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res = 0xff;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        break;
    case ESP_GATTS_READ_EVT:
        res = find_char_and_desr_index(p_data->read.handle);
        if (res == SPP_IDX_SPP_STATUS_VAL)
        {
            // TODO:client read the status characteristic
        }
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        res = find_char_and_desr_index(p_data->write.handle);
        if (p_data->write.is_prep == false)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
            if (res == SPP_IDX_SPP_COMMAND_VAL)
            {
                uint8_t *spp_cmd_buff = NULL;
                spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                if (spp_cmd_buff == NULL)
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                    break;
                }
                memset(spp_cmd_buff, 0x0, (spp_mtu_size - 3));
                memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
                xQueueSend(cmd_cmd_queue, &spp_cmd_buff, 10 / portTICK_PERIOD_MS);
            }
            else if (res == SPP_IDX_SPP_DATA_NTF_CFG)
            {
                if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00))
                {
                    enable_data_ntf = true;
                }
                else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00))
                {
                    enable_data_ntf = false;
                }
            }
            else if (res == SPP_IDX_SPP_DATA_RECV_VAL)
            {
#ifdef SPP_DEBUG_MODE
                esp_log_buffer_char(GATTS_TABLE_TAG, (char *)(p_data->write.value), p_data->write.len);
#else
                uart_write_bytes(UART_NUM_0, (char *)(p_data->write.value), p_data->write.len);
#endif
            }
            else
            {
                // TODO:
            }
        }
        else if ((p_data->write.is_prep == true) && (res == SPP_IDX_SPP_DATA_RECV_VAL))
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            store_wr_buffer(p_data);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
        if (p_data->exec_write.exec_write_flag)
        {
            print_write_buffer();
            free_write_buffer();
        }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;
    case ESP_GATTS_CONF_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
#ifdef SUPPORT_HEARTBEAT
        uint16_t cmd = 0;
        xQueueSend(cmd_heartbeat_queue, &cmd, 10 / portTICK_PERIOD_MS);
#endif
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        enable_data_ntf = false;
#ifdef SUPPORT_HEARTBEAT
        enable_heart_ntf = false;
        heartbeat_count_num = 0;
#endif
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n", param->add_attr_tab.num_handle);
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != SPP_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
        }
        else
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
        }
        break;
    }
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                if (spp_profile_tab[idx].gatts_cb)
                {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

// 任务句柄，包含创建任务的所有状态，对任务的操作都通过操作任务句柄实现
TaskHandle_t led_task_Handler = NULL;
// lcd任务句柄
TaskHandle_t spi_lcd_task_Handler = NULL;
//妊娠预测任务句柄
TaskHandle_t pregnancy_predict_task_Handler = NULL;

// led_task 任务，控制LED闪烁
void led_task(void *arg)
{
    // 配置LED为推挽输出，设置初始电平为0

    led_init(LED, 0);
    led_init(LONG_WAVE_LED, 0);
    led_init(SHORT_WAVE_LED, 0);
    led_init(RED_LED, 0);
    ledc_pwm_hs_init();
    // ledc_pwm_set_duty_cycle(ledc_hs_ch, 0, 500);
    while (1)
    {
        if (beep.beep_status)
        {
            for (uint8_t i = 0; i < beep.beep_num; i++)
            {
                ledc_pwm_set_duty_cycle(ledc_hs_ch, 0, 500);
                vTaskDelay(beep.beep_delay / portTICK_PERIOD_MS);
                ledc_pwm_set_duty_cycle(ledc_hs_ch, 0, 0);
                vTaskDelay(beep.beep_delay / portTICK_PERIOD_MS);
            }
            beep.beep_num = 1;
            beep.beep_delay = 200;
            beep.beep_status = 0;
        }
        else
        {
            // LED状态闪烁
            led_blink(LED);
            vTaskDelay(5 / portTICK_PERIOD_MS);
            // vTaskSuspend(led_task_Handler); // 暂停任务，LED停止闪烁
        }
    }
}

// key_scan_task 任务，单独运行一个任务，运行按键检测（滤除按键抖动、短按、长按）
void key_scan_task(void *arg)
{
    //按键检测任务
    key_scan(3, BOOT, KEY_USR, PRESS_SENSOR); // 3个按键
}
uint16_t arr_in[6] = {0};
uint8_t preg_num = 0, npreg_num = 0;
uint8_t preg_predict_status = 0; //预测状态标志
uint8_t preg_predict_num = 0;    //预测次数
#define PREG_PREDICT_NUM 30
uint8_t ble_send_buff[20] = {0};
// key_catch_task 任务。去捕获按键事件，并控制LED任务状态。
void key_catch_task(void *arg)
{
    uint32_t key_event = 0;
    uint32_t key_gpio = 0; // 触发按键事件的按键gpio序号
    uint32_t key_type = 0; // 触发按键事件的类型
    // SD卡初始化、FATFS文件系统挂载。总线使用SPI模式，20MHz。
    sdmmc_card_t *card = sd_card_fatfs_spi_init();
    while (!card)
    { // 验证错误，如果返回值为0，则SD卡初始化及FATFS挂载失败，重试
        ESP_LOGE(TAG, "Failed !! %d Retry!!", false);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        card = sd_card_fatfs_spi_init();
        sprintf(lcd.lcd_title_str, "SdCard Init Failed!!");
        beep.beep_status = 1;
        lcd.lcd_refresh_status = 1; //刷新LCD屏幕
    }
    // ADC1及输入通道初始化
    //不同的衰减系数，会影响经校准补偿后的值。且衰减越大，对读数准确性的影响也越大。
    //不同的衰减系数，对应的ESP32引脚输入范围为：0：75~1008mV。2.5：78~1317mV。6：107~1821mV。11：142~3108mV。
    //超过衰减系数的电压输入范围，可能导致ADC损毁。可以通过设置更高的衰减来扩展输入范围，但推荐使用更低的衰减以提高校准后值的准确性。
    adc1_init_with_calibrate(ADC_ATTEN_DB_11, 2, ADC_CHANNEL_4, ADC_CHANNEL_5);
    adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
    adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);

    while (1)
    {
        // 以阻塞方式，不断读取 key_evt_queue 队列信息，监控按键事件
        if (xQueueReceive(key_evt_queue, &key_event, portMAX_DELAY))
        {
            // 拆分按键事件，得到按键值和按键类型
            key_gpio = key_event & 0x0000FFFF;                      // 按键值
            key_type = key_event >> 16;                             // 按键类型（1为短按，2为长按）
            ESP_LOGW(TAG, "key:%d, type:%d\n", key_gpio, key_type); // 输出按键事件
            if (key_gpio == BOOT)
            { // 使用 KEY 控制LED运行状态
                if (key_type == 1)
                { // 1，短按，暂停LED任务
                    // vTaskSuspend(led_task_Handler); // 暂停任务，LED停止闪烁
                    // led_set(LED, 0);
                    if (data_save.data_sampling_status) //判断是否采样完成
                    {
                        // data_save.data_save_status = 1; //保存数据标志置1
                        // data_save.data_save_response = 1;
                        data_save.data_sampling_status = 0;
                        ESP_LOGI(TAG, "Wating Save Data");
                        // 使用POSIX和C标准库函数来处理文件。
                        ESP_LOGI(TAG, "Saving Data");
                        // data_save.data_save_response = 0;
                        // 首先创建一个文件。
                        ESP_LOGI(TAG, "Opening file");
                        FILE *f = fopen(MOUNT_POINT "/sampling.csv", "a"); //无改文件则创建 有则直接写 并在文件末尾添加 不覆盖源文件
                        if (f == NULL)
                        {
                            ESP_LOGE(TAG, "Failed to open file for writing");
                            while (1)
                            { // 验证错误，如果返回值为0，则SD卡初始化及FATFS挂载失败，重试
                                ESP_LOGE(TAG, "Failed writing!!");
                                sprintf(lcd.lcd_title_str, "Failed writing!!");
                                beep.beep_status = 1;
                                beep.beep_delay = 1000;
                                vTaskDelay(200 / portTICK_PERIOD_MS);
                                lcd.lcd_refresh_status = 1; //刷新LCD屏幕
                            }
                            return;
                        }
                        // 将字符串写入到文件中 写入顺序为
                        //母兔编号-母兔妊娠状态-母兔生产状态-母兔妊娠天数-探头选择-红光(PD1)-红光(PD2)-短波(PD1)-短波(PD2)-长波(PD1)-长波(PD2)
                        fprintf(f, "%d,%d, %d,%d,%d,%d,%d,%d,%d,%d,%d\n", rabbit.rabbit_number, rabbit.preg_status, rabbit.preg_produce_status,
                                rabbit.preg_day, work_mode.probe_choose, pd_value.re_pd_1, pd_value.re_pd_2, pd_value.sw_pd_1, pd_value.sw_pd_2, pd_value.lw_pd_1, pd_value.lw_pd_2);
                        // 关闭文件
                        fclose(f);
                        ESP_LOGI(TAG, "File Written");
                        sprintf(lcd.lcd_title_str, "File Written");
                        rabbit.rabbit_number++;
                        lcd.lcd_refresh_status = 1; //刷新LCD屏幕
                        beep.beep_status = 1;
                    }
                    else
                    {
                        switch (key.key_choose)
                        {
                        case 0:
                            /* code */
                            work_mode.mode_choose++;
                            if (work_mode.mode_choose == 3)
                            {
                                work_mode.mode_choose = 0;
                            }
                            ble_commend_buff.device_status[2] = 0xc0;
                            ble_commend_buff.device_status[4] = work_mode.mode_choose;
                            if (is_connected)
                            {
                                beep.beep_status = 1; //蜂鸣器
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], ble_commend_buff.device_status[1], ble_commend_buff.device_status, false);
                            }
                            break;
                        case 1: //母兔妊娠状态
                            /* code */
                            if (work_mode.mode_choose == 2)
                            {                                                     //是否在预测模式下 该键为剃毛状态
                                rabbit.cut_hair_status = !rabbit.cut_hair_status; //
                                ble_commend_buff.device_status[2] = 0xc1;
                                ble_commend_buff.device_status[4] = rabbit.cut_hair_status;
                                if (is_connected)
                                {
                                    beep.beep_status = 1; //蜂鸣器
                                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], ble_commend_buff.device_status[1], ble_commend_buff.device_status, false);
                                }
                            }
                            else
                            {
                                beep.beep_status = 1; //蜂鸣器
                                rabbit.preg_status = !rabbit.preg_status;
                            }
                            break;
                        case 2:                   //母兔生产状态
                            beep.beep_status = 1; //蜂鸣器
                            rabbit.preg_produce_status = !rabbit.preg_produce_status;
                            break;
                        case 3:                   //母兔怀孕天数 限制在15天
                            beep.beep_status = 1; //蜂鸣器
                            rabbit.preg_day++;
                            if (rabbit.preg_day == 15)
                            {
                                rabbit.preg_day = 10;
                            }
                            break;
                        case 4:                   //探头显示
                            beep.beep_status = 1; //蜂鸣器
                            work_mode.probe_choose++;
                            if (work_mode.probe_choose == 5)
                            {
                                work_mode.probe_choose = 1;
                            }
                            break;
                        default:
                            break;
                        }

                        lcd.lcd_refresh_status = 1; //屏幕刷新
                    }
                }
                else if (key_type == 2)
                {
                }
            }
            else if (key_gpio == KEY_USR)
            {

                if (key_type == 1)
                { // 1，短按，暂停LED任务
                    // LCD_ShowString(80 - 1, 20 - 1, YELLOW, BLUE, "Hello!", 12, 0);
                    if (data_save.data_sampling_status) //判断是否采样完成
                    {
                        // data_save.data_save_status = 0; //保存数据标志置1
                        // data_save.data_save_response = 1;
                        data_save.data_sampling_status = 0;
                        beep.beep_status = 1;
                        sprintf(lcd.lcd_title_str, "Don't Saving Data");
                        lcd.lcd_refresh_status = 1; //刷新LCD屏幕
                    }
                    else
                    {
                        key.key_choose++;
                        if (key.key_choose == key.key_num)
                        {
                            key.key_choose = 0;
                        }
                        ble_commend_buff.device_status[2] = 0xc3;
                        ble_commend_buff.device_status[4] = key.key_choose;
                        if (is_connected)
                        {
                            beep.beep_status = 1; //蜂鸣器
                            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], ble_commend_buff.device_status[1], ble_commend_buff.device_status, false);
                        }

                        lcd.lcd_refresh_status = 1; //屏幕刷新
                    }
                }
                else if (key_type == 2)
                { // 2，长按，继续LED任务
                    // LCD_Clear(LGRAYBLUE);
                    if (work_mode.test_flag)
                    { //停止测试采样

                        work_mode.test_flag = 0;
                    }
                }
            }
            else if (key_gpio == PRESS_SENSOR)
            {
                if (key_type == 1)
                {
                    switch (work_mode.mode_choose)
                    {
                    case 0 /* constant-expression */: //测试模式
                                                      /* code */
                        while (1)
                        {
                            if (work_mode.test_flag)
                            {
                                if (is_connected)
                                {                           //是否连接蓝牙
                                    led_on(SHORT_WAVE_LED); //开启短波LED
                                    vTaskDelay(1 / portTICK_PERIOD_MS);
                                    pd_value.sw_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
                                    pd_value.sw_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
                                    led_off(SHORT_WAVE_LED); //关闭LED
                                    led_on(LONG_WAVE_LED);   //开启长波LED
                                    vTaskDelay(1 / portTICK_PERIOD_MS);
                                    pd_value.lw_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
                                    pd_value.lw_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
                                    led_off(LONG_WAVE_LED); //关闭LED
                                    led_on(RED_LED);        //开启红光LED
                                    vTaskDelay(1 / portTICK_PERIOD_MS);
                                    pd_value.re_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
                                    pd_value.re_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
                                    led_off(RED_LED); //关闭红光LED
                                    //蓝牙发送数据
                                    memset(ble_send_buff, 0, sizeof(uint8_t) * 4);
                                    sprintf((char *)ble_send_buff, "re %d %d", pd_value.re_pd_1, pd_value.re_pd_2);
                                    ESP_LOGI(GATTS_TABLE_TAG, "%s", ble_send_buff);
                                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 20, ble_send_buff, false);
                                    vTaskDelay(40 / portTICK_PERIOD_MS); //
                                    memset(ble_send_buff, 0, sizeof(uint8_t) * 4);
                                    sprintf((char *)ble_send_buff, "sw %d %d", pd_value.sw_pd_1, pd_value.sw_pd_2);
                                    ESP_LOGI(GATTS_TABLE_TAG, "%s", ble_send_buff);
                                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 20, ble_send_buff, false);
                                    vTaskDelay(40 / portTICK_PERIOD_MS); //
                                    memset(ble_send_buff, 0, sizeof(uint8_t) * 4);
                                    sprintf((char *)ble_send_buff, "lw %d %d", pd_value.lw_pd_1, pd_value.lw_pd_2);
                                    ESP_LOGI(GATTS_TABLE_TAG, "%s", ble_send_buff);
                                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 20, ble_send_buff, false);
                                    vTaskDelay(40 / portTICK_PERIOD_MS); //
                                }
                                else
                                {
                                    beep.beep_status = 1;
                                    sprintf(lcd.lcd_title_str, "ble is not connect");
                                    lcd.lcd_refresh_status = 1; //刷新LCD
                                    uint8_t delay_time = 1000 / work_mode.test_fre;
                                    vTaskDelay(delay_time / portTICK_PERIOD_MS); //
                                }
                            }
                        }
                        break;
                    case 1:                                  //采样模式
                        if (!data_save.data_sampling_status) //采样完成标志
                        {
                            beep.beep_status = 1;
                            led_on(SHORT_WAVE_LED); //开启短波LED
                            vTaskDelay(1 / portTICK_PERIOD_MS);
                            pd_value.sw_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
                            pd_value.sw_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
                            led_off(SHORT_WAVE_LED); //关闭LED
                            led_on(LONG_WAVE_LED);   //开启长波LED
                            vTaskDelay(1 / portTICK_PERIOD_MS);
                            pd_value.lw_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
                            pd_value.lw_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
                            led_off(LONG_WAVE_LED); //关闭LED
                            led_on(RED_LED);        //开启红光LED
                            vTaskDelay(1 / portTICK_PERIOD_MS);
                            pd_value.re_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
                            pd_value.re_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
                            led_off(RED_LED); //关闭红光LED
                            sprintf(lcd.lcd_title_str, "Is Saving Data?");
                            lcd.lcd_refresh_status = 1;         //刷新LCD
                            data_save.data_sampling_status = 1; //采样完成标志置1
                        }
                        break;
                    case 2: //预测模式
                        if (preg_predict_status == 0&&beep.beep_status ==0)
                        {
                            beep.beep_status = 1;
                            beep.beep_delay = 100;
                            preg_predict_status = 1;
                        }
                        break;
                    default:
                        break;
                    }
                }
                else if (key_type == 2)
                {
                    // work_mode.test_flag = 0;
                    // beep.beep_status = 1;
                }
            }
        }
    }
}
//母兔妊娠预测任务
//根据剃毛状态选择不同的预测SVM
//每次预测 采集10次数据分别预测 并得出结果
//根据10次结果中妊娠与未妊娠的次数决定最终结果
//并将最终结果发送至上位机

void pregnancy_predict_task(void *arg)
{
    for (;;)
    {
        if (preg_predict_status)
        {
            preg_predict_num++;
            ble_commend_buff.device_status[2] = 0xc2; //妊娠状态地址
            //是否连接蓝牙
            led_on(SHORT_WAVE_LED); //开启短波LED
            vTaskDelay(1 / portTICK_PERIOD_MS);
            arr_in[2] = pd_value.sw_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
            arr_in[3] = pd_value.sw_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
            led_off(SHORT_WAVE_LED); //关闭LED
            led_on(LONG_WAVE_LED);   //开启长波LED
            vTaskDelay(1 / portTICK_PERIOD_MS);
            arr_in[4] = pd_value.lw_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
            arr_in[5] = pd_value.lw_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
            led_off(LONG_WAVE_LED); //关闭LED
            led_on(RED_LED);        //开启红光LED
            vTaskDelay(1 / portTICK_PERIOD_MS);
            arr_in[0] = pd_value.re_pd_1 = adc1_cal_get_voltage_mul(ADC_CHANNEL_5, ADC_DUPLICATE_READ_NUM);
            arr_in[1] = pd_value.re_pd_2 = adc1_cal_get_voltage_mul(ADC_CHANNEL_4, ADC_DUPLICATE_READ_NUM);
            led_off(RED_LED); //关闭红光LED
                              //蓝牙发送数据
            if (is_connected)
            {
                memset(ble_send_buff, 0, sizeof(uint8_t) * 4);
                sprintf((char *)ble_send_buff, "re %d %d", pd_value.re_pd_1, pd_value.re_pd_2);
                ESP_LOGI(GATTS_TABLE_TAG, "%s", ble_send_buff);
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 20, ble_send_buff, false);
                vTaskDelay(40 / portTICK_PERIOD_MS); //
                memset(ble_send_buff, 0, sizeof(uint8_t) * 4);
                sprintf((char *)ble_send_buff, "sw %d %d", pd_value.sw_pd_1, pd_value.sw_pd_2);
                ESP_LOGI(GATTS_TABLE_TAG, "%s", ble_send_buff);
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 20, ble_send_buff, false);
                vTaskDelay(40 / portTICK_PERIOD_MS); //
                memset(ble_send_buff, 0, sizeof(uint8_t) * 4);
                sprintf((char *)ble_send_buff, "lw %d %d", pd_value.lw_pd_1, pd_value.lw_pd_2);
                ESP_LOGI(GATTS_TABLE_TAG, "%s", ble_send_buff);
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 20, ble_send_buff, false);
                vTaskDelay(40 / portTICK_PERIOD_MS); //
            }
            if (svm_prediction(arr_in, rabbit.cut_hair_status) == 1)
            {
                preg_num++; //妊娠结果数量加1
                ble_commend_buff.device_status[4] = 0x01;
                if (is_connected)
                { //蓝牙发送状态
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], ble_commend_buff.device_status[1], ble_commend_buff.device_status, false);
                }
            }
            else if (svm_prediction(arr_in, rabbit.cut_hair_status) == -1)
            {
                npreg_num++; //未妊娠结果数量加1
                ble_commend_buff.device_status[4] = 0x00;
                if (is_connected)
                { //蓝牙发送状态
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], ble_commend_buff.device_status[1], ble_commend_buff.device_status, false);
                }
            }
            else
            {
            }
            // vTaskDelay(1000 / portTICK_PERIOD_MS); //延时1s

            if (preg_predict_num == PREG_PREDICT_NUM)
            {
                beep.beep_status = 1; //蜂鸣器开启
                beep.beep_num = 3;    //蜂鸣次数
                beep.beep_delay = 500;    //蜂鸣延时
                if (preg_num > npreg_num)
                {
                    //母兔妊娠
                    ble_commend_buff.device_status[4] = 0x01;
                    if (is_connected)
                    { //蓝牙发送状态
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], ble_commend_buff.device_status[1], ble_commend_buff.device_status, false);
                    }
                }
                else if (preg_num < npreg_num)
                {
                    //母兔未妊娠
                    ble_commend_buff.device_status[4] = 0x00;
                    if (is_connected)
                    { //蓝牙发送状态
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], ble_commend_buff.device_status[1], ble_commend_buff.device_status, false);
                    }
                }
                else
                {
                    //重新采集
                    ble_commend_buff.device_status[4] = 0x02;
                    if (is_connected)
                    { //蓝牙发送状态
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], ble_commend_buff.device_status[1], ble_commend_buff.device_status, false);
                    }
                }
                preg_num = 0;
                npreg_num = 0;
                preg_predict_num = 0;
            }
            preg_predict_status = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
// spi_lcd_task 任务。初始化 SPI3-LCD，并显示图形
void spi_lcd_task(void *arg)
{

    // 配置SPI3-主机模式，配置DMA通道、DMA字节大小，及 MISO、MOSI、CLK的引脚。
    spi_master_init(VSPI_HOST, LCD_DEF_DMA_CHAN, LCD_DMA_MAX_SIZE, SPI3_DEF_PIN_NUM_MISO, SPI3_DEF_PIN_NUM_MOSI, SPI3_DEF_PIN_NUM_CLK);
    // lcd-驱动IC初始化（注意：普通GPIO最大只能30MHz，而IOMUX默认的SPI引脚，CLK最大可以设置到80MHz）（注意排线不要太长，高速时可能会花屏）
    spi_lcd_init(VSPI_HOST, 60 * 1000 * 1000, LCD_SPI3_DEF_PIN_NUM_CS0);

    // // 配置I2C0-主机模式，100K，指定 SCL-22，SDA-21
    // i2c_master_init(I2C_NUM_1, 100000, GPIO_NUM_22, GPIO_NUM_21);
    // // CW2015初始化
    // cw_bat_init();
    // // 画实心矩形
    // LCD_DrawFillRectangle(35 - 1, 8 - 1, 36 - 1, 8 - 1, BLUE);

    while (1)
    {
        if (lcd.lcd_refresh_status)
        {
            // 清屏，用单一底色
            LCD_Clear(WHITE);
            // // 画空心矩形
            LCD_DrawRectangle(0, 0, 160 - 1, 80 - 1, BLACK);
            LCD_DrawRectangle(0, 0, 160 - 1, 20 - 1, BLACK);
            LCD_DrawRectangle(0, 60 - 1, 32 - 1, 80 - 1, BLACK);
            LCD_DrawRectangle(32 - 1, 60 - 1, 74 - 1, 80 - 1, BLACK);
            LCD_DrawRectangle(74 - 1, 60 - 1, 116 - 1, 80 - 1, BLACK);
            LCD_DrawRectangle(116 - 1, 60 - 1, 138 - 1, 80 - 1, BLACK);
            LCD_DrawRectangle(138 - 1, 60 - 1, 160 - 1, 80 - 1, BLACK);
            //显示采样数据
            LCD_ShowString(2 - 1, 2 - 1, YELLOW, BLUE, lcd.lcd_title_str, 12, 1);
            LCD_ShowString(5 - 1, 26 - 1, YELLOW, BLUE, "PD1", 12, 1); //短波PD1数值
            LCD_ShowString(5 - 1, 43 - 1, YELLOW, BLUE, "PD2", 12, 1);
            LCD_ShowString(30 - 1, 35 - 1, YELLOW, BLUE, "RE", 14, 1);
            LCD_ShowNum(45 - 1, 26 - 1, RED, YELLOW, pd_value.re_pd_1, 4, 12, 1);
            LCD_DrawLine(45 - 1, 40 - 1, 65 - 1, 40 - 1, RED);
            LCD_ShowNum(45 - 1, 43 - 1, RED, YELLOW, pd_value.re_pd_2, 4, 12, 1); //短波PD2数值

            LCD_ShowString(75 - 1, 35 - 1, YELLOW, BLUE, "SW", 14, 1);
            LCD_ShowNum(90 - 1, 26 - 1, RED, YELLOW, pd_value.sw_pd_1, 4, 12, 1); //长波PD1数值
            LCD_DrawLine(90 - 1, 40 - 1, 110 - 1, 40 - 1, RED);
            LCD_ShowNum(90 - 1, 43 - 1, RED, YELLOW, pd_value.sw_pd_2, 4, 12, 1); //长波PD2数值

            LCD_ShowString(115 - 1, 35 - 1, YELLOW, BLUE, "LW", 14, 1);
            LCD_ShowNum(130 - 1, 26 - 1, RED, YELLOW, pd_value.lw_pd_1, 4, 12, 1); //长波PD1数值
            LCD_DrawLine(130 - 1, 40 - 1, 150 - 1, 40 - 1, RED);
            LCD_ShowNum(130 - 1, 43 - 1, RED, YELLOW, pd_value.lw_pd_2, 4, 12, 1); //长波PD2数值
            //显示母兔状态
            if (work_mode.mode_choose == 1)
            { //采样模式 显示母兔编号
                LCD_ShowNum(120 - 1, 2 - 1, YELLOW, BLUE, rabbit.rabbit_number, 4, 12, 1);
            }
            LCD_ShowString(2 - 1, 64 - 1, YELLOW, BLUE, work_mode.mode_choose == 0 ? "test" : work_mode.mode_choose == 1 ? "samp"
                                                                                                                         : "pred",
                           12, key.key_choose == 0 ? 0 : 1);
            LCD_ShowString(34 - 1, 64 - 1, YELLOW, BLUE, rabbit.preg_status ? "PREG" : "NPREG", 12, key.key_choose == 1 ? 0 : 1);
            LCD_ShowString(76 - 1, 64 - 1, YELLOW, BLUE, rabbit.preg_produce_status ? "PRODU" : "HOUBEI", 12, key.key_choose == 2 ? 0 : 1);
            LCD_ShowNum(118 - 1, 64 - 1, YELLOW, BLUE, rabbit.preg_day, 2, 12, key.key_choose == 3 ? 0 : 1);
            LCD_ShowNum(140 - 1, 64 - 1, YELLOW, BLUE, work_mode.probe_choose, 2, 12, key.key_choose == 4 ? 0 : 1);
        }
        lcd.lcd_refresh_status = 0;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        // ESP_LOGI(TAG, "LCD_TASK RUNING");
        // LCD_Clear(LGRAYBLUE);
        // printf("cw2015 version %d\n",cw_version());
        // printf("cw2015 soc %d\n", cw_soc());
        // printf("cw2015 voltage %d\n",cw_get_vol());
    }
}

void app_main(void)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    spp_task_init();

    // 创建 led_task 任务，任务栈空间大小为 2048，任务优先级为3（configMINIMAL_STACK_SIZE 不能用 printf，会导致ESP32反复重启）
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, &led_task_Handler);

    // 创建 spi_lcd_task 任务。
    xTaskCreate(spi_lcd_task, "spi_lcd_task", 4096, NULL, 4, &spi_lcd_task_Handler);

    // 创建 key_scan_task 任务，运行任务栈空间大小为 4096，任务优先级为3。实测分配小于2048会导致开机反复重启
    xTaskCreate(key_scan_task, "key_scan_task", 4096, NULL, 3, NULL);
    // 创建 key_catch_task 任务，任务栈空间大小为 4096，任务优先级为3。去捕获按键事件，并控制LED任务状态。
    xTaskCreate(key_catch_task, "key_catch_task", 4096, NULL, 2, NULL);
    // 创建 pregnancy_predict_task 任务，任务栈空间大小为 4096，任务优先级为3。预测母兔妊娠状态任务
    xTaskCreate(pregnancy_predict_task, "pregnancy_predict_task", 4096, NULL, 3, NULL);
}
