#include "compass.h"
#include <stdio.h>

#include "math.h"
//#define DEBUG

#ifdef DEBUG
#define PRINT printf
#else
void dummy_print(){return;}
#define PRINT dummy_print
#endif

I2C_HandleTypeDef *_i2c = 0;

bool compass_init(I2C_HandleTypeDef *i2c)
{
	uint8_t data[16] = {0};
	bool success = false;
	//store our handle
	_i2c = i2c;

	HAL_I2C_Mem_Read(_i2c, HMC5883L_ADDRESS, HMC5883L_REG_IDENT_A, 1, data, 3, 100);
	PRINT("ID = %02x, %02x, %02x\n\r", data[0], data[1], data[2]);
	if((data[0] == HMC5883L_REG_IDENT_A_VAL) &&
	  (data[1] == HMC5883L_REG_IDENT_B_VAL) &&
	  (data[2] == HMC5883L_REG_IDENT_C_VAL))
	{
		PRINT("Compass Found\n\r");
		//configure the compass
		data[0] = HMC5883L_REG_CONFIG_A;
		data[1] = (HMC5883L_SAMPLES_8 << HMC5883L_CRA_MA_Offset) | (HMC5883L_DATARATE_15HZ << HMC5883L_CRA_DO_Offset);
		data[2] = HMC5883L_RANGE_1_3GA << HMC5883L_CRB_GN_Offset;
		data[3] = HMC5883L_CONTINOUS;
		HAL_I2C_Master_Transmit(_i2c, HMC5883L_ADDRESS,data, 4, 100);
	}
	return success;
}


int16_t compass_get_heading()
{
	typedef struct {
		int16_t x;
		int16_t z;
		int16_t y;
	} data_t;

	data_t mag_data = {0};

	if(HAL_OK == HAL_I2C_Mem_Read(_i2c, HMC5883L_ADDRESS,HMC5883L_REG_OUT_X_M, 1, (uint8_t *)&mag_data, 6,100))
	{
		mag_data.x = ((mag_data.x >> 8)&0x00FF) | ((mag_data.x << 8) & 0xFF00);
		mag_data.y = ((mag_data.y >> 8)&0x00FF) | ((mag_data.y << 8) & 0xFF00);
		mag_data.z = ((mag_data.z >> 8)&0x00FF) | ((mag_data.z << 8) & 0xFF00);
		PRINT("X=%d ; Y=%d ; Z=%d\n\r", mag_data.x, mag_data.y, mag_data.z);

		double xy = 0.0;
		double heading = 0.0;
		if(mag_data.y != 0)
		{
			xy = (double)mag_data.x / (double)mag_data.y;
			xy = atan(xy) * 180.0 / 3.14;
			if(mag_data.y < 0)
			{
				heading = 270 - xy;
			}
			else
			{
				heading = 90 - xy;
			}
		}
		else
		{
			if(mag_data.x < 0)
				heading = 180.0;
			else
				heading = 0.0;
		}
		PRINT("%d degrees \n\r", (uint16_t)heading);
		return (uint16_t)heading;
	}//if hal ok
	return -1;
}

