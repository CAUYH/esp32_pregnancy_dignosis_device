// cw2015_ds电量计驱动函数
#include "cw2015.h"
#include "i2c_config.h"

//@param reg 寄存器地址
//@param p_data 数据
uint8_t CW2015_Write(uint8_t reg, uint8_t p_data)
{
	i2c_master_write_slave_reg(I2C_NUM_1, W2015_W_ADDR, reg, &p_data, 1, CW2015_TICKS_TO_WAIT);
	return 0;
}

uint8_t CW2015_Read(uint8_t reg, uint8_t *p_data)
{
	i2c_master_read_slave_reg(I2C_NUM_1, W2015_R_ADDR, reg, p_data, 1, CW2015_TICKS_TO_WAIT);
	return 0;
}
uint8_t CW2015_Read_word(unsigned char point_reg, unsigned char *r_pdata, unsigned int length)
{
	unsigned char count;
	for (count = 0; count < length; count++)
	{
		if (count == 0)
		{
			i2c_master_read_slave_reg(I2C_NUM_1, W2015_R_ADDR, point_reg, r_pdata, 1, CW2015_TICKS_TO_WAIT);
		}
		else
		{
			i2c_master_read_slave(I2C_NUM_1, W2015_R_ADDR, r_pdata, 1, CW2015_TICKS_TO_WAIT);
		}
		if ((count + 1) < length)
		{
			r_pdata++;
		}
	}
	return 0;
}

// uint8_t CW2015_Read_word(unsigned char point_reg,unsigned char *r_pdata, unsigned int length)
// {
// 	unsigned char count;

// 	 IIC_Start();
// 	 IIC_Send_Byte(W2015_W_ADDR);       //����������ַ+д����
// 	if(IIC_Wait_Ack())
// 	{
// 		return 1;
// 	}
// 	 IIC_Send_Byte(point_reg);
// 	if(IIC_Wait_Ack())
// 	{
// 		return 1;
// 	}
// 	 IIC_Start();
// 	 IIC_Send_Byte(W2015_R_ADDR);    //�����ڼ��ַ+������
// 	if(IIC_Wait_Ack())
// 	{
// 		return 1;
// 	}
// 	for(count = 0; count < length; count++ ){
// 		*r_pdata = IIC_Read_Byte(0);
// 		if(count + 1 < length){
// 			r_pdata++;
// 			IIC_Ack();
// 		}
// 	}
// 	IIC_Nack();
// 	 IIC_Stop();
// 	return 0;
// }

//读CW的版本号 默认返回值0x6f
uint8_t cw_version(void)
{
	uint8_t reg_val;
	CW2015_Read(CW_VERSION, &reg_val);
	return reg_val;
}
//
//读剩余电量%
uint8_t cw_soc(void)
{
	uint8_t reg_val;
	CW2015_Read(CW_SOC_MSB, &reg_val);
	return reg_val;
}

//读剩余电量1/256% 高精度
uint8_t cw_soc_accurate(void)
{
	uint8_t reg_val;
	CW2015_Read(CW_SOC_LSB, &reg_val);
	return reg_val;
}
//读配置寄存器
uint8_t cw_config_read(void)
{
	uint8_t reg_val;
	CW2015_Read(CW_CONFIG, &reg_val);
	return reg_val;
}
//读模式寄存器
uint8_t cw_mode_read(void)
{
	uint8_t reg_val;
	CW2015_Read(CW_MODE, &reg_val);
	return reg_val;
}

//开启cw2015

uint8_t cw_start(void)
{
	CW2015_Write(CW_MODE, 0x00);
	return 0;
}
// cw2015睡眠

uint8_t cw_sleep(void)
{
	CW2015_Write(CW_MODE, 0xC0);
	return 0;
}
//写配置寄存器
uint8_t cw_config_write(void)
{
	return 0;
}

uint8_t CHARGE = 0; //是否接充电器标志位，1为接充电器，0为拔出充电器
unsigned int allow_charger_always_zero = 0;
unsigned char if_quickstart = 0;
unsigned char reset_loop = 0;

/*定义一个全局变量，外部文件要使用时请先include Cellwise CW201x Driver for MCU.h文件，再用extern声明cw_bat*/
struct STRUCT_CW_BATTERY cw_bat;

/*这个函数的作用是更新ic内的电池profile信息，一般只有在ic VDD掉电后再上电时才执行
return 1 : i2c读写错， return 2 : 芯片处于sleep模式 return 3 : 写入的profile信息读出后与代码中的不一致*/
unsigned char cw_update_config_info(void)
{
	uint8_t ret = 0;
	unsigned char i;
	unsigned char reset_val;
	unsigned char reg_val;
	/* make sure no in sleep mode */
	ret = CW2015_Read(CW_MODE, &reg_val);
	if (ret)
	{
		return 1;
	}
	if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP)
	{
		return 2;
	}
	/* update new battery info */
	for (i = 0; i < SIZE_BATINFO; i++)
	{
		reg_val = cw_bat_config_info[i];
		ret = CW2015_Write(CW_BATINFO + i, reg_val);
		if (ret)
		{
			return 1;
		}
	}

	/* readback & check */
	for (i = 0; i < SIZE_BATINFO; i++)
	{
		ret = CW2015_Read(CW_BATINFO + i, &reg_val);
		if (ret)
		{
			return 1;
		}
		if (reg_val != cw_bat_config_info[i])
		{
			return 3;
		}
	}
	/* set cw2015/cw2013 to use new battery info */
	ret = CW2015_Read(CW_CONFIG, &reg_val);
	if (ret)
	{
		return 1;
	}
	reg_val |= CONFIG_UPDATE_FLG; /* set UPDATE_FLAG */
	reg_val &= 0x07;			  /* clear ATHD */
	reg_val |= ATHD;			  /* set ATHD */
	ret = CW2015_Write(CW_CONFIG, reg_val);
	if (ret)
	{
		return 1;
	}
	/* reset */
	reset_val = MODE_NORMAL;
	reg_val = MODE_RESTART;
	ret = CW2015_Write(CW_MODE, &reg_val);
	if (ret)
	{
		return 1;
	}
	ret = CW2015_Write(CW_MODE, &reset_val);
	if (ret)
	{
		return 1;
	}
	vTaskDelay(1 / portTICK_PERIOD_MS);
	return 0;
}

/*电量计初始化函数 每次开机后要执行
return 1 : i2c读写错， return 2 : 芯片处于sleep模式 return 3 : 写入的profile信息读出后与代码中的不一致 return 4 : 芯片启动后30s内读电量值一直异常*/
unsigned char cw_init(void)
{
	unsigned ret;
	unsigned char i;
	unsigned char reg_val;

	/* wake up cw2015/13 from sleep mode */
	ret = CW2015_Write(CW_MODE, MODE_NORMAL);
	if (ret)
	{
		return 1;
	}

	/* check ATHD if not right */
	ret = CW2015_Read(CW_CONFIG, &reg_val);
	if (ret)
	{
		return 1;
	}
	if ((reg_val & 0xf8) != ATHD)
	{
		//"the new ATHD need set"
		reg_val &= 0x07; /* clear ATHD */
		reg_val |= ATHD; /* set ATHD */
		ret = CW2015_Write(CW_CONFIG, &reg_val);
		if (ret)
		{
			return 1;
		}
	}

	/* check config_update_flag if not right */
	ret = CW2015_Read(CW_CONFIG, &reg_val);
	if (ret)
	{
		return 1;
	}
	if (!(reg_val & CONFIG_UPDATE_FLG))
	{
		//"update flag for new battery info need set"
		ret = cw_update_config_info();
		if (ret)
		{
			return ret;
		}
	}
	else
	{
		for (i = 0; i < SIZE_BATINFO; i++)
		{
			ret = CW2015_Read(CW_BATINFO + i, &reg_val);
			if (ret)
			{
				return 1;
			}
			if (cw_bat_config_info[i] != reg_val)
			{
				break;
			}
		}
		if (i != SIZE_BATINFO)
		{
			//"update flag for new battery info need set"
			ret = cw_update_config_info();
			if (ret)
			{
				return ret;
			}
		}
	}
	/* check SOC if not eqaul 255 */
	for (i = 0; i < 30; i++)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		ret = CW2015_Read(CW_SOC_MSB, &reg_val);
		if (ret)
			return 1;
		else if (reg_val <= 100)
			break;
	}

	if (i >= 30)
	{
		ret = CW2015_Write(CW_MODE, MODE_SLEEP);
		// "cw2015/cw2013 input unvalid power error_2\n";
		return 4;
	}
	return 0;
}

#ifdef BAT_LOW_INTERRUPT
/*当一次alrt 事件到来时，cw2015 ic会拉低arlt pin产生中断，这时需要对06寄存器的最高uint8_t位清0，才能让cw2015 ic释放alrt pin 下面函数的作用是释放alrt pin*/
unsigned char cw_release_alrt_pin(void)
{
	signed char ret = 0;
	unsigned int reg_val;
	unsigned char alrt;

	ret = CW2015_Read(REG_RRT_ALERT, &reg_val);
	if (ret)
	{
		return -1;
	}
	alrt = reg_val & 0x80;

	reg_val = reg_val & 0x7f;
	ret = CW2015_Write(REG_RRT_ALERT, &reg_val);
	if (ret)
	{
		return -1;
	}

	return alrt;
}

/*下面的函数是我写的一个例子，例子函数的作用是更新新的低电告警值为上次的 -1， 比如我们的代码开始的时候设定的低电告警值是10，那当电量降到10后*/
/*主控处理完中断后，我把新的低电告警值9写在了对应的寄存器中。 ATHD是08寄存器的前5个uint8_t*/
uint8_t cw_update_athd()
{
	uint8_t ret = 0;
	unsigned char reg_val;
	char new_athd = 0;

	ret = CW2015_Read(REG_CONFIG, &reg_val);
	if (ret)
	{
		return -1;
	}
	new_athd = (reg_val >> 3) - 1;
	if (new_athd <= 0)
	{
		new_athd = 0;
	}
	new_athd = new_athd << 3;

	//"the new ATHD need set"
	reg_val &= 0x07;	 /* clear ATHD */
	reg_val |= new_athd; /* set new ATHD */
	ret = CW2015_Write(REG_CONFIG, &reg_val);
	if (ret)
	{
		return -1;
	}
	return 0;
}

static void ALRT_ISR() interrupt
{
	/*User can do something when alrt */
	/*客户可以在这里加入当中断到来时想做的操作*/
	cw_release_alrt_pin();
	cw_update_athd();
	/*User can write new alrt to CONFIG resiger*/
}
#endif

uint8_t cw_por(void)
{
	uint8_t ret = 0;

	ret = CW2015_Write(CW_MODE, MODE_SLEEP);
	if (ret)
		return -1;
	vTaskDelay(1 / portTICK_PERIOD_MS);

	ret = CW2015_Write(CW_MODE, MODE_NORMAL);
	if (ret)
		return -1;
	vTaskDelay(1 / portTICK_PERIOD_MS);

	ret = cw_init();
	if (ret)
		return ret;
	return 0;
}

int cw_get_capacity(void)
{
	uint8_t ret = 0;
	unsigned char allow_capacity;
	unsigned char reg_val;
	// unsigned char reset_val;
	unsigned char cw_capacity;
	// int charge_time;

	ret = CW2015_Read(CW_SOC_MSB, &reg_val);
	if (ret)
	{
		return -1;
	}

	cw_capacity = reg_val;
	/*假设ic出现问题，读取电量不在合理值范围内5次，重启ic。如果中间读到正确的值，那么5次的计数器清0，正确显示*/
	if ((cw_capacity < 0) || (cw_capacity > 100))
	{
		// "get cw_capacity error; cw_capacity = %d\n"
		reset_loop++;
		if (reset_loop > 5)
		{
			ret = cw_por(); // por ic
			if (ret)
				return -1;
			reset_loop = 0;
		}
		return cw_bat.capacity;
	}
	else
	{
		reset_loop = 0;
	}

	/*ic出错了，充了很久一直还是0%，一般我们用半个小时，那么重启下ic*/
	if ((cw_bat.usb_online > 0) && (cw_capacity == 0))
	{
		allow_charger_always_zero++;
		if ((allow_charger_always_zero >= BATTERY_DOWN_MIN_CHANGE_SLEEP) && (if_quickstart == 0))
		{
			ret = cw_por(); // por ic
			if (ret)
			{
				return -1;
			}
			if_quickstart = 1;
			allow_charger_always_zero = 0;
		}
	}
	else if ((if_quickstart == 1) && (cw_bat.usb_online == 0))
	{
		if_quickstart = 0;
	}

	return (cw_capacity);
}

unsigned int cw_get_vol(void)
{
	uint8_t ret = 0;
	unsigned char get_ad_times = 0;
	unsigned char reg_val[2] = {0, 0};
	unsigned long ad_value = 0;
	unsigned int ad_buff = 0;
	unsigned int ad_value_min = 0;
	unsigned int ad_value_max = 0;

	for (get_ad_times = 0; get_ad_times < 3; get_ad_times++)
	{
		ret = CW2015_Read_word(CW_VCELL_MSB, &reg_val[0], 2);
		if (ret)
		{
			return 1;
		}
		ad_buff = (reg_val[0] << 8) + reg_val[1];

		if (get_ad_times == 0)
		{
			ad_value_min = ad_buff;
			ad_value_max = ad_buff;
		}
		if (ad_buff < ad_value_min)
		{
			ad_value_min = ad_buff;
		}
		if (ad_buff > ad_value_max)
		{
			ad_value_max = ad_buff;
		}
		ad_value += ad_buff;
	}
	ad_value -= ad_value_min;
	ad_value -= ad_value_max;
	ad_value = ad_value * 305 / 1000;
	return (ad_value); // 14位ADC转换值
}

void update_capacity(void)
{
	int cw_capacity;
	cw_capacity = cw_get_capacity();
	if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat.capacity != cw_capacity))
	{
		cw_bat.capacity = cw_capacity;
	}
}

void update_vol(void)
{
	unsigned int cw_voltage;
	cw_voltage = cw_get_vol();
	if (cw_voltage == 1)
	{
		// read voltage error
		//		cw_bat.voltage = cw_bat.voltage;
	}
	else if (cw_bat.voltage != cw_voltage)
	{
		cw_bat.voltage = cw_voltage;
	}
}

/*
static void update_alt(void)
{
	signed int alt;
	alt = cw_get_alt();
	if ((rrt >= 0) && (cw_bat.alt != alt))
	{
		cw_bat.alt = (unsigned int)alt;
	}
}
*/

void update_usb_online(void)
{
	if (CHARGE == 1)
	//这里请注意，这里是客户需要自己配置修改的地方
	//请修改代码保证DC插入时配置cw_bat.usb_online为 1，DC不在时配置cw_bat.usb_online为0
	{
		cw_bat.usb_online = 1;
	}
	else
	{
		cw_bat.usb_online = 0;
	}
}

////////////////////////////////////////MCU一秒调用一次//////////////////////////////////////////
void cw_bat_work(void)
{
	update_usb_online();
	update_capacity();
	update_vol();
}

/*
static void cw_bat_gpio_init(void)
{

	 usb_det_pin -- init
	 alt_pin  -- init

	 return 0;
}
*/

///////////////////////////////////////MCU开机初始化时调用.//////////////////////////////////////
unsigned char cw_bat_init(void)
{
	unsigned char ret;
	unsigned char loop = 0;


	ret = cw_init();
	while ((loop++ < 3) && (ret != 0))
	{
		ret = cw_init();
	}

	cw_bat.usb_online = 0;
	cw_bat.capacity = 2;
	cw_bat.voltage = 0;
	cw_bat.alt = 0;

	return ret;
}
