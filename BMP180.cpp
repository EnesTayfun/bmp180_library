/*
 *  BMP180.cpp
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

#include "BMP180.h"

void BMP180::Init(){
	Read_Calibration_Data();
}

void BMP180::Read_Calibration_Data(){

	HAL_I2C_Mem_Read(I2C_UNIT, BMP180_ADDRESS, AC1_MSB_REG, 1, eeprom, 22, HAL_MAX_DELAY);

	Cal.AC1 = ((eeprom[0] << 8) | eeprom[1]);
	Cal.AC2 = ((eeprom[2] << 8) | eeprom[3]);
	Cal.AC3 = ((eeprom[4] << 8) | eeprom[5]);
	Cal.AC4 = ((eeprom[6] << 8) | eeprom[7]);
	Cal.AC5 = ((eeprom[8] << 8) | eeprom[9]);
	Cal.AC6 = ((eeprom[10] << 8) | eeprom[11]);
	Cal.B1 = ((eeprom[12] << 8) | eeprom[13]);
	Cal.B2 = ((eeprom[14] << 8) | eeprom[15]);
	Cal.MB = ((eeprom[16] << 8) | eeprom[17]);
	Cal.MC = ((eeprom[18] << 8) | eeprom[19]);
	Cal.MD = ((eeprom[20] << 8) | eeprom[21]);
}

void BMP180::Get_Raw_Temperature(){

	data = TEMPERATURE_KEY;
	HAL_I2C_Mem_Write(I2C_UNIT, BMP180_ADDRESS, CTRL_MEAS_REG, 1, &data, 1, 100);
	HAL_Delay (5);  // wait 4.5 ms
	HAL_I2C_Mem_Read(I2C_UNIT, BMP180_ADDRESS, OUT_MSB_REG, 1, raw_temp, 2, 100);
	Cal.UT = ((raw_temp[0]<<8) + raw_temp[1]);
}

void BMP180::Get_Raw_Pressure(int16_t oss){
	data = PRESSURE_OSS_1_KEY + (oss<<6);
	HAL_I2C_Mem_Write(I2C_UNIT, BMP180_ADDRESS, CTRL_MEAS_REG, 1, &data, 1, 1000);
	switch (oss)
	{
		case (0):
			HAL_Delay (5);
			break;
		case (1):
			HAL_Delay (8);
			break;
		case (2):
			HAL_Delay (14);
			break;
		case (3):
			HAL_Delay (26);
			break;
	}
	HAL_I2C_Mem_Read(I2C_UNIT, BMP180_ADDRESS, OUT_MSB_REG, 1, raw_press, 3, 1000);
	Cal.UP = (((raw_press[0]<<16)+(raw_press[1]<<8)+raw_press[2]) >> (8-oss));
}

void BMP180::Calc_Temperature(){
	Get_Raw_Temperature();
	Cal.X1 = ((Cal.UT-Cal.AC6) * (Cal.AC5/(pow(2,15))));
	Cal.X2 = ((Cal.MC*(pow(2,11))) / (Cal.X1+Cal.MD));
	Cal.B5 = Cal.X1+Cal.X2;
	temperature = (Cal.B5+8)/(pow(2,4))/10;
}

void BMP180::Calc_Pressure(int16_t oss){
	Get_Raw_Pressure(oss);
	Calc_Temperature();
	Cal.B6 = Cal.B5-4000;
	Cal.X1 = (Cal.B2 * (Cal.B6*Cal.B6/(pow(2,12))))/(pow(2,11));
	Cal.X2 = Cal.AC2*Cal.B6/(pow(2,11));
	Cal.X3 = Cal.X1+Cal.X2;
	Cal.B3 = ((((int32_t)Cal.AC1*4+Cal.X3)<<oss)+2)/4;
	Cal.X1 = Cal.AC3*Cal.B6/pow(2,13);
	Cal.X2 = (Cal.B1 * (Cal.B6*Cal.B6/(pow(2,12))))/(pow(2,16));
	Cal.X3 = ((Cal.X1+Cal.X2)+2)/pow(2,2);
	Cal.B4 = Cal.AC4*(uint32_t)(Cal.X3+32768)/(pow(2,15));
	Cal.B7 = ((uint32_t)Cal.UP-Cal.B3)*(50000>>oss);
	if (Cal.B7<0x80000000) pressure = (Cal.B7*2)/Cal.B4;
	else pressure = (Cal.B7/Cal.B4)*2;
	Cal.X1 = (pressure/(pow(2,8)))*(pressure/(pow(2,8)));
	Cal.X1 = (Cal.X1*3038)/(pow(2,16));
	Cal.X2 = (-7357*pressure)/(pow(2,16));
	pressure = pressure + (Cal.X1+Cal.X2+3791)/(pow(2,4));
}


void BMP180::Calc_Altitude(int16_t oss){
	Calc_Pressure (oss);
	altitude = 44330*(1-(pow((pressure/(float)atmtopa), 0.19029495718)));
}

float BMP180::Show_Temperature(){
	Calc_Temperature();
	return temperature;
}


float BMP180::Show_Pressure(BMP180_Oss_Reg_t oss){
	Calc_Pressure(oss);
	return pressure;
}

float BMP180::Show_Altitude(BMP180_Oss_Reg_t oss){
	Calc_Altitude(oss);
	return altitude;
}
