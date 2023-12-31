/*==============================================================================

 Name:           PositionControl.c
 Description:    functionality of camera position control driver
 Copyright:      Geierwally, 2020(c)

==============================================================================*/
#include "PositionControl.h"
#include "timer.h"
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <math.h>

int fd = 0;
systemtimer zRotTimer;	// for z rotation angle calculation
unsigned long long zRotTime = 100000; // rotation time z axis for angle calculation 150 ms
const int ACCEL_OFFSET   = 200;
const int GYRO_OFFSET    = -14;  // 151
const float GYRO_SENSITITY = 16.4;  // 131 is sensivity of gyro from data sheet
const float GYRO_SCALE   = 1; //  0.02 by default - tweak as required
float LOOP_TIME    = 0.1; // 0.1 = 100ms
int accValue[3], accAngle[3], temperature, accCorr;
float gyroValue[3];
float gyroAngle[3], gyroCorr;
const int accelAddress[3] = {ACCEL_XOUT_H,ACCEL_YOUT_H,ACCEL_ZOUT_H};
const int gyroAddress[3] = {GYRO_XOUT_H,GYRO_YOUT_H,GYRO_ZOUT_H};


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PC_Init(void)
{
	LOOP_TIME    = (float) zRotTime / 1000000.0;
	fd = wiringPiI2CSetup(Device_Address);          /*Initializes I2C with device Address*/
    // MPU6050 init
	wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
	wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
	wiringPiI2CWriteReg8 (fd, CONFIG, 0);		    /* Write to Configuration register */
	wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	    /* Write to Gyro Configuration register */
	wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/* Write to interrupt enable register */
}

short read_raw_data(int addr)
{
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}

float distance(float a, float b)
{
	return(sqrt(a * a) + (b * b));
}

float get_y_rotation(float x,float y, float z)
{
    float degrees = atan2(x, distance(y,z))* 57.2957795;
    return (degrees);
}

float get_x_rotation(float x, float y, float z)
{
    float degrees = atan2(y, distance(x,z)) * 57.2957795;
    return(degrees);
}

float get_z_rotation(float x, float y, float z)
{
    float degrees = atan2(z, distance(x,y)) * 57.2957795;
    return(degrees);
}

void  readSensorData (void)
{
	float accCorr = 0;

	for(int i=0; i<3; i++)
    {
		accValue[i] = read_raw_data(accelAddress[i]);
		accCorr = accValue[i] - ACCEL_OFFSET;
		accCorr = map(accCorr,-16800, 16800, -90, 90);
		accAngle[i] = constrain(accCorr, -90, 90);
		gyroValue[i] = read_raw_data(gyroAddress[i]);
		gyroCorr = ((gyroValue[i] - GYRO_OFFSET)/GYRO_SENSITITY );
	    gyroAngle[i] += (gyroCorr * GYRO_SCALE) * LOOP_TIME;
    }

	/* Divide raw value by sensitivity scale factor  not used because set direct to scale*/
    //	Acc_x = accValue[0]/16384.0;
    //	Acc_y = accValue[1]/16384.0;
    //	Acc_z = accValue[2]/16384.0;
}


void PC_Test (void)
{
	systemtimer printTimer;
    readSensorData();
    startMeasurement(&printTimer);
    startMeasurement(&zRotTimer);
	while(1)
	{
		if(isExpired(zRotTime,&zRotTimer))
		{
			readSensorData();
			startMeasurement(&zRotTimer);
		}
		if(isExpired(500000,&printTimer))
		{
			startMeasurement(&printTimer);
    	    printf("\nrotx=%d grad\troty=%d grad\trotz=%.3fd grad\n",accAngle[0],accAngle[1],gyroAngle[2]);
		}
	}

}
