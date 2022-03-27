/*
library name: 	MPU6050 6 axis module
written by: 		T.Jaber
Date Written: 	25 Mar 2019
Last Modified: 	20 April 2019 by Mohamed Yaqoob
Description: 		MPU6050 Module Basic Functions Device Driver library that use HAL libraries.
References:			
								- MPU6050 Registers map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
								- Jeff Rowberg MPU6050 library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
								
* Copyright (C) 2019 - T. Jaber
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.

*/

//Header files
#include "TJ_MPU6050.h"
//Library Variable
//1_ Handle 
static I2C_HandleTypeDef i2cHandler;
MPU_ConfigTypeDef *MpuConfig;
//2_ Accel & Gyro Scaling Factor
float accelScalingFactor, gyroScalingFactor;
RawData_Def Accel;
RawData_Def Gyro;
float accel_reg_bias[3];
float gyr_reg_bias[3];
//Fucntion Definitions
//1- i2c Handler 
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd, MPU_ConfigTypeDef *mpuCon)
{
	//Copy I2C CubeMX handle to local library
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));
	//MPU config handle to local library
	MpuConfig = mpuCon;
}

//2- i2c Read
HAL_StatusTypeDef I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{
	uint8_t MPUADDR;
	uint8_t i2cBuf[2] = {0};
	//Need to Shift address to make it proper to i2c operation
	MPUADDR = (MPU_ADDR<<1);
	i2cBuf[0] = ADDR;
	if(HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cBuf, 1, 5)!= HAL_OK) return HAL_ERROR;
	if(HAL_I2C_Master_Receive(&i2cHandler, MPUADDR, i2cBif, NofData, 5)!= HAL_OK) return HAL_ERROR;
	return HAL_OK;
}

//3- i2c Write
HAL_StatusTypeDef I2C_Write8(uint8_t ADDR, uint8_t data)
{
	uint8_t i2cData[2] = {0};
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t MPUADDR = (MPU_ADDR<<1);
	if(HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cData, 2,5)!= HAL_OK) return HAL_ERROR;
	return HAL_OK;
}

//4- MPU6050 Initialaztion Configuration 
void MPU6050_Config(void)
{
	uint8_t Buffer = 0;
	float PI = 3.14159f;
	//Clock Source 
	//Reset Device
	I2C_Write8(PWR_MAGT_1_REG, 0x80);
	HAL_Delay(100);
	Buffer = MpuConfig ->ClockSource & 0x07; //change the 7th bits of register
	Buffer |= (MpuConfig ->Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	I2C_Write8(PWR_MAGT_1_REG, Buffer);
	HAL_Delay(100); // should wait 10ms after changing the clock setting.
	
	//Set the Digital Low Pass Filter
	Buffer = 0;
	Buffer = MpuConfig->CONFIG_DLPF & 0x07;
	I2C_Write8(CONFIG_REG, Buffer);
	
	//Select the Gyroscope Full Scale Range
	Buffer = 0;
	Buffer = (MpuConfig->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(GYRO_CONFIG_REG, Buffer);
	
	//Select the Accelerometer Full Scale Range 
	Buffer = 0; 
	Buffer = (MpuConfig->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(ACCEL_CONFIG_REG, Buffer);
	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(0x04);
	
	//Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling Factor
	switch (MpuConfig->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = 2.00f/32768.00f;
			break;
		
		case AFS_SEL_4g:
			accelScalingFactor = 4.00f/32768.00f;
				break;
		
		case AFS_SEL_8g:
			accelScalingFactor = 8.00f/32768.00f;
			break;
		
		case AFS_SEL_16g:
			accelScalingFactor = 16.00f/32768.00f;
			break;
		
		default:
			break;
	}
	//Gyroscope Scaling Factor 
	switch (MpuConfig->Gyro_Full_Scale)
	{
		case FS_SEL_250:
			gyroScalingFactor = (250.0f/32768.0f)*PI/180;
			break;
		
		case FS_SEL_500:
				gyroScalingFactor = (500.0f/32768.0f)*PI/180;
				break;
		
		case FS_SEL_1000:
			gyroScalingFactor = (1000.0f/32768.0f)*PI/180;
			break;
		
		case FS_SEL_2000:
			gyroScalingFactor = (2000.0f/32768.0f)*PI/180;
			break;
		
		default:
			break;
	}
	
}

//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}

//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
	I2C_Write8(SMPLRT_DIV_REG, SMPRTvalue);
}

//7- Get External Frame Sync.
uint8_t MPU6050_Get_FSYNC(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(CONFIG_REG, &Buffer, 1);
	Buffer &= 0x38; 
	return (Buffer>>3);
}

//8- Set External Frame Sync. 
void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync)
{
	uint8_t Buffer = 0;
	I2C_Read(CONFIG_REG, &Buffer,1);
	Buffer &= ~0x38;
	
	Buffer |= (ext_Sync <<3); 
	I2C_Write8(CONFIG_REG, Buffer);	
}

//9- Get Accel Raw Data
HAL_StatusTypeDef ReadI2C_MPU(void)
{
	uint8_t AcceArr[6];
	uint8_t GyroArr[6];
	if(I2C_Read(ACCEL_XOUT_H_REG, AcceArr,6)== HAL_OK && I2C_Read(GYRO_XOUT_H_REG, GyroArr,6) == HAL_OK)
	{
		//Accel Raw Data
		Accel.x = (int16_t)((AcceArr[0]<<8) | AcceArr[1]);
		Accel.y = (int16_t)((AcceArr[2]<<8) | AcceArr[3]);
		Accel.z = (int16_t)((AcceArr[4]<<8) | AcceArr[5]);
		//Gyro Raw Data
		Gyro.x = (int16_t)((GyroArr[0]<<8) | GyroArr[1]);
		Gyro.y = (int16_t)((GyroArr[2]<<8) | GyroArr[3]);
		Gyro.z = (int16_t)((GyroArr[4]<<8) | GyroArr[5]);
	}
	else
	{
		HAL_I2C_Init(&i2cHandler);
		MPU6050_Config();
		return HAL_ERROR;
	}
	return HAL_OK;
}

//10 - Set offset
HAL_StatusTypeDef SetOffset(float accelOffset[3], float gyroOffset[3])
{
	uint8_t dataG[6];
	uint8_t dataA[6];
	uint8_t check = 0;
	uint8_t AcceArr[6];

	I2C_Read(0x06, AcceArr, 6);	
	
	for(int i = 0; i<3; i++)
	{
		// calculate Acc offset
		accelOffset[i] /= 8.0f;
		// calculate Gyr offset
		gyroOffset[i] /= 4.0f;
		check = 0;
		
		// Set Accel offset
		uint8_t mask_bit = 0;
		uint16_t mask = 0x0001;
		int16_t accel_cur_bias = (int16_t)(AcceArr[2*i]<<8) | AcceArr[2*i+1];
		if(accel_cur_bias & mask) mask_bit = 0x01;
		accel_cur_bias += round(accelOffset[i]);
		dataA[2*i] = (accel_cur_bias >> 8) & 0xff;
		if(I2C_Write8(0x06 + 2*i, dataA[2*i]) == HAL_OK) check++;
		dataA[2*i + 1] = accel_cur_bias & 0xff;
		dataA[2*i + 1] = dataA[2*i + 1]|mask_bit;
		if(I2C_Write8(0x07 + 2*i, dataA[2*i + 1]) == HAL_OK) check++;
		
		// Set Gyro offset
		int16_t tempGyroOffset = (int16_t)round(gyroOffset[i]);
		dataG[2*i] = (tempGyroOffset>>8)&0xff;
		if(I2C_Write8(0x13 + 2*i, dataG[2*i]) == HAL_OK) check++;
		dataG[2*i + 1] = tempGyroOffset&0xff;
		if(I2C_Write8(0x14 + 2*i, dataG[2*i + 1]) == HAL_OK) check++;
		
		// Check error
		if(check < 4) 
		{
			HAL_I2C_Init(&i2cHandler);
			MPU6050_Config();
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

//11- Calibrate MPU6050
HAL_StatusTypeDef CalibrateMPU6050(void)
{
	float accelOffset[3] = {0}, gyroOffset[3] = {0};
	int32_t at[3] = {0}, wt[3] = {0};
	uint16_t count = 0;
	for(int i = 0; i<20000; i++)
	{
		if(ReadI2C_MPU() == HAL_OK)
		{
			at[0] += Accel.x;
			at[1] += Accel.y;
			at[2] += (Accel.z - 16384);
			wt[0] += Gyro.x;
			wt[1] += Gyro.y;
			wt[2] += Gyro.z;
			count++;
		}
	}
	if(count < 15000) return HAL_ERROR;

	for(int i = 0; i < 3; i++){
		// calculate Acc offset
		accelOffset[i] = -(at[i]*1.0000f)/count;
		// calculate Gyr offset
		gyroOffset[i] = -(wt[i]*1.0000f)/count;
	}
	
	Soft_SetOffset(accelOffset, gyroOffset);
	
	return HAL_OK;
}

//12 - Set offset Soft
void Soft_SetOffset(float accelOffset[3], float gyroOffset[3])
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.TypeErase  = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_11;
	EraseInitStruct.NbSectors    = 1;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
	for(int i = 0; i < 3; i++)
	{
		accel_reg_bias[i] = *((float*)((__IO uint32_t *)(startAddressFlash+4*i)));
		if(isnan(accel_reg_bias[i])) accel_reg_bias[i] = 0;
		accel_reg_bias[i] += accelOffset[i];
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddressFlash + 4*i, *((uint32_t*)&accel_reg_bias[i]));
		gyr_reg_bias[i] = *((float*)((__IO uint32_t *)(startAddressFlash+4*i+12)));
		if(isnan(gyr_reg_bias[i])) gyr_reg_bias[i] = 0;
		gyr_reg_bias[i] += gyroOffset[i];
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddressFlash + 4*i + 12, *((uint32_t*)&gyr_reg_bias[i]));
	}
	HAL_FLASH_Lock();
}

//13 - Get offset Soft
void Soft_GetOffset(void)
{
	for(int i = 0; i < 3; i++)
	{
		accel_reg_bias[i] = *((float*)((__IO uint32_t *)(startAddressFlash + 4*i)));
		if(isnan(accel_reg_bias[i])) accel_reg_bias[i] = 0;
		gyr_reg_bias[i] = *((float*)((__IO uint32_t *)(startAddressFlash + 4*i + 12)));
		if(isnan(gyr_reg_bias[i])) gyr_reg_bias[i] = 0;
	}
}

//14- Measure errors
float* Errors_Measure(void)
{
	uint16_t N = 10000;
	float ztb[6] = {0};
	static float rs[12] = {0};
	int32_t at[3] = {0}, wt[3] = {0};
	int16_t Ax[10000], Ay[10000] = {0}, Az[10000] = {0};
	int16_t Wx[10000] = {0}, Wy[10000] = {0}, Wz[10000] = {0};
	float P[6] = {0};
	uint16_t count = 0;
	for(int i = 0; i < N; i++)
	{
		if(ReadI2C_MPU() == HAL_OK)
		{
			Ax[i] = Accel.x;
			Ay[i] = Accel.y;
			Az[i] = (Accel.z - 16384);
			Wx[i] = Gyro.x;
			Wy[i] = Gyro.y;
			Wz[i] = Gyro.z;
			at[0] += Ax[i];
			at[1] += Ay[i];
			at[2] += Az[i];
			wt[0] += Wx[i];
			wt[1] += Wy[i];
			wt[2] += Wz[i];
			count++;
		}
	}
	if(count < 0.7*N) return 0;

	// Mean -> bias
	ztb[0] = (at[0]/count + accel_reg_bias[0])*accelScalingFactor; ztb[1] = (at[1]/count + accel_reg_bias[1])*accelScalingFactor;
	ztb[2] = (at[2]/count + accel_reg_bias[2])*accelScalingFactor;
	ztb[3] = (wt[0]/count + gyr_reg_bias[0])*gyroScalingFactor; ztb[4] = (wt[1]/count + gyr_reg_bias[1])*gyroScalingFactor;
	ztb[5] = (wt[2]/count + gyr_reg_bias[2])*gyroScalingFactor;

	// Standard deviation
	for(int i = 0; i < N; i++)
	{
		P[0] += (pow((Ax[i] + accel_reg_bias[0])*accelScalingFactor - ztb[0],2));
		P[1] += (pow((Ay[i] + accel_reg_bias[1])*accelScalingFactor - ztb[1],2));
		P[2] += (pow((Az[i] + accel_reg_bias[2])*accelScalingFactor - ztb[2],2));
		P[3] += (pow((Wx[i] + gyr_reg_bias[0])*gyroScalingFactor - ztb[3],2));
		P[4] += (pow((Wy[i] + gyr_reg_bias[1])*gyroScalingFactor - ztb[4],2));
		P[5] += (pow((Wz[i] + gyr_reg_bias[2])*gyroScalingFactor - ztb[5],2));
	}

	for(int i = 0; i < 6; i++)
	{
		P[i] /= count;
		P[i] = sqrt(P[i]);
	}

	memcpy(rs,ztb,sizeof(ztb));
	memcpy(&rs[6],P,sizeof(P));

	return rs;
}


