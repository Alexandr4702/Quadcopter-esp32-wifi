/*
 * Mpu9250.h
 *
 *  Created on: Aug 6, 2020
 *      Author: alexandr
 */

#ifndef MAIN_MPU9250_H_
#define MAIN_MPU9250_H_

#include "stdint.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "SPIbus.h"


class vec3f
{
public:
	float x;
	float y;
	float z;
	vec3f():x(0),y(0),z(0)
	{

	}
	vec3f(vec3f& other):x(other.x),y(other.y),z(other.z)
	{

	}
	vec3f(vec3f&& other):x(other.x),y(other.y),z(other.z)
	{

	}
	vec3f(float x_, float y_, float z_):x(x_), y(y_), z(z_)
	{

	}
	friend vec3f operator + (const vec3f& other1,const vec3f& other2)
	{
		return vec3f(other1.x + other2.x,other1.y + other2.y,other1.z + other2.z);
	}
	friend vec3f operator * (const vec3f& other1,const vec3f& other2)
	{
		return vec3f(other1.x * other2.x, other1.y * other2.y, other1.z * other2.z);
	}
	friend vec3f operator * (const vec3f& other1,float consT)
	{
		return vec3f(other1.x * consT, other1.y * consT, other1.z * consT);
	}
	vec3f operator = (const vec3f& other1)
	{
		return vec3f(other1.x,other1.y,other1.z);
	}
	vec3f operator = (const int16_t* data)
	{
		return vec3f(static_cast <float> (data[0]), static_cast <float> (data[1]), static_cast <float> (data[2]));
	}
	vec3f operator *= (float const_)
	{
		return vec3f(this->x * const_, this->y * const_, this->z * const_);
	}

};



class Mpu9250 {
public:
	Mpu9250();
	virtual ~Mpu9250();
	void init();
private:
	SPI_t &mySPI = vspi;
    spi_device_handle_t device=0;

	int16_t raw_data_accel[3] = {0};
	int16_t raw_data_gyro[3] = {0};
	int16_t raw_data_mag[3] = {0};
	int16_t raw_temp = 0;
	uint8_t rx_buffer[20] = {0};

	float lsb_to_mg_accel = 0;
	float lsb_to_dps_gyro = 0;
	float lsb_to_nT_mag   = 0;

	vec3f accel;
	vec3f mag;
	vec3f anguar_velo;
	float temp=0;

private:
	void write_reg(uint8_t reg, uint8_t data);
	uint8_t read_reg(uint8_t reg);
	void read_raw_data();
	void delay(uint32_t delay_);
public:
	void read_data();

	vec3f getAccel() {
		return accel;
	}

	vec3f getAnguarVelo() {
		return anguar_velo;
	}

	vec3f getMag() {
		return mag;
	}
};

#endif /* MAIN_MPU9250_H_ */
