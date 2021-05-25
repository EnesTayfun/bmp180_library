/*
 *  BMP180.h
 *
 *  Created on: Apr 25, 2021
 *      Author: Enes Tayfun CICEK
 *
 * 	Copyright (C) 2021 enescicek.com
 *
 * 	This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
 *	of the GNU General Public License version 3 as published by the Free Software Foundation.
 * 	This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
 * 	or indirectly by this software, read more about this on the GNU General Public License.
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32l4xx_hal.h"  //You can choose which STM32 family you are using
#include <math.h>

extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
#define I2C_UNIT &hi2c1   // You can choose which I2C unit you are using, example &hti2cx --> x = 1, 2, 3...

typedef enum{
	BMP180_ADDRESS 	   =    0x77<<1U,  //Device 7 - Bit I2C Address 0x77
	CTRL_MEAS_REG	   =	0xF4U,
	OUT_MSB_REG		   =	0xF6U,
	ID_REG			   =	0xD0U,
	AC1_MSB_REG		   =    0xAAU,
}BMP180_Addresses_t;

typedef enum{
	TEMPERATURE_KEY		=	0x2EU,
	PRESSURE_OSS_1_KEY	=	0x34U,
	PRESSURE_OSS_2_KEY	=	0x74U,
	PRESSURE_OSS_3_KEY	=	0xB4U,
	PRESSURE_OSS_4_KEY	=	0xF4U
}BMP180_Key_Values_t;

typedef enum{
	SAMPLE_RATIO_1		=	0x00U,
	SAMPLE_RATIO_2		=	0x01U,
	SAMPLE_RATIO_4		=	0x02U,
	SAMPLE_RATIO_8		=	0x03U
}BMP180_Oss_Reg_t;

typedef struct{
	int16_t				AC1,
					AC2,
					AC3,
					B1,
					B2,
					MB,
					MC,
					MD;

	uint16_t			AC4,
					AC5,
					AC6;

	int32_t				UT,
					UP,
					X1,
					X2,
					X3,
					B3,
					B5,
					B6;

	uint32_t	 		B4,
					B7;

}BMP180_Calib_t;

class BMP180{
private:
	//float temperature, pressure, altitude;

	void Calc_Altitude(int16_t oss);
	void Read_Calibration_Data(void);
	void Get_Raw_Temperature(void);
	void Get_Raw_Pressure(int16_t oss);
	void Calc_Temperature(void);
	void Calc_Pressure(int16_t oss);

public:
	float temperature, pressure, altitude;
	uint8_t eeprom[22] = {0};
	uint8_t raw_temp[2] = {0};
	uint8_t raw_press[3] = {0};
	uint8_t data;
	const int atmtopa = 101325;  // 1 atm is 101325 pascal
	BMP180_Calib_t Cal;

	void Init();
	float Show_Temperature(void);
	float Show_Pressure(BMP180_Oss_Reg_t oss);
	float Show_Altitude(BMP180_Oss_Reg_t oss);

};

#endif /* INC_BMP180_H_ */
