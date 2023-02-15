/*
 * cw2015.h
 *
 *  Created on: Mar 1, 2022
 *      Author: yuanhao
 */

#ifndef INC_CW2015_H_
#define INC_CW2015_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define CW2015_TICKS_TO_WAIT           (100 / portTICK_RATE_MS)    // I2C读写的超时等待时间

#define W2015_R_ADDR      0xC5            //读地址
#define W2015_W_ADDR      0xC4            //写地址

//各功能寄存器地址
#define CW_VERSION     0x00         //Returns IC version, software version  R default value: 0x6F
#define CW_VCELL_MSB  0x02         //Report 14-bit A/D measurement of battery voltage R default value: 0x0
#define CW_VCELL_LSB  0x03
#define CW_SOC_MSB 0x04           //Report 16-bit SOC result calculated R 0x00
#define CW_SOC_LSB 0x05
#define CW_RRT_ALRT_MSB  0x06     //13 bits remaining run time and low SOC alert bit W/R 0x00
#define CW_RRT_ALRT_MSB  0x07
#define CW_CONFIG 0x08           //Configure register, alert threshold set W/R 0x18
#define CW_MODE 0x0A             //Special command for IC state W/R 0xC0
#define CW_BATINFO             0x10
uint8_t cw_version(void);
uint8_t cw_soc(void);
unsigned int cw_get_vol(void);
uint8_t cw_soc_accurate(void);
uint8_t cw_config_read(void);
uint8_t cw_mode_read(void);
uint8_t cw_start(void);
uint8_t cw_sleep(void);
uint8_t cw_config_write(void);
unsigned char cw_bat_init(void);
uint8_t CW2015_Write(uint8_t reg, uint8_t p_data);
uint8_t CW2015_Read(uint8_t reg, uint8_t *p_data);
uint8_t CW2015_Read_word(unsigned char point_reg,unsigned char *r_pdata, unsigned int length);

#define	READ_CW2015				0xc5
#define	WRITE_CW2015			0xc4
#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)
#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        //ATHD = 0%
#define SIZE_BATINFO        64
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 1800      // the min time allow battery change quantity when run 30min
//#define BAT_LOW_INTERRUPT    1
/*电池建模信息，客户拿到自己电池匹配的建模信息后请替换*/
static unsigned char cw_bat_config_info[SIZE_BATINFO] = {
0x15  ,0x4C  ,0x5D  ,0x5D  ,0x5A  ,0x59  ,0x55  ,
0x51  ,0x4E  ,0x48  ,0x46  ,0x41  ,0x3C  ,0x39  ,
0x33  ,0x2D  ,0x25  ,0x1E  ,0x19  ,0x19  ,0x1A  ,
0x2C  ,0x44  ,0x4A  ,0x43  ,0x40  ,0x0C  ,0xCD  ,
0x22  ,0x43  ,0x56  ,0x82  ,0x78  ,0x6F  ,0x62  ,
0x60  ,0x42  ,0x19  ,0x37  ,0x31  ,0x00  ,0x1D  ,
0x59  ,0x85  ,0x8F  ,0x91  ,0x91  ,0x18  ,0x58  ,
0x82  ,0x94  ,0xA5  ,0xFF  ,0xAF  ,0xE8  ,0xCB  ,
0x2F  ,0x7D  ,0x72  ,0xA5  ,0xB5  ,0xC1  ,0x46  ,
0xAE
};

//****************************struct*********************************/
struct STRUCT_CW_BATTERY {
	unsigned char usb_online;
	unsigned int capacity;
	unsigned int voltage;
	unsigned char alt;
};
extern struct STRUCT_CW_BATTERY  cw_bat;



#ifdef __cplusplus
}
#endif
#endif /* INC_CW2015_H_ */
