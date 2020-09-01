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
#include <Eigen/Dense>

#include "freertos/task.h"



class Mpu9250 {
public:
	Mpu9250();
	virtual ~Mpu9250();
	void init();
	void init_spi();
private:
	SPI_t &mySPI = vspi;
    spi_device_handle_t device_accel_gyro=0;


	volatile int16_t raw_data_accel[3] = {0};
	volatile int16_t raw_data_gyro[3] = {0};
	volatile int16_t raw_data_mag[3] = {0};
	volatile int16_t raw_temp = 0;
	uint8_t rx_buffer[20] = {0};

	float lsb_to_mg_accel = 0;
	float lsb_to_dps_gyro = 0;
	float lsb_to_nT_mag   = 0;
	float temp=0;

	Eigen::Vector3f accel;
	Eigen::Vector3f anguar_velo;
	Eigen::Vector3f mag;
private:
	void write_reg(uint8_t reg, uint8_t data);
	uint8_t read_reg(uint8_t reg);
	void read_regs (uint8_t * data,uint16_t size);
	void read_raw_data();
	void delay(uint32_t delay_);
public:
	void read_data();

	const Eigen::Vector3f& getAccel() const {
		return accel;
	}

	const Eigen::Vector3f& getAnguarVelo() const {
		return anguar_velo;
	}

	const Eigen::Vector3f& getMag() const {
		return mag;
	}
private:
	const uint8_t SMPLRT_DIV     = 25;
	const uint8_t CONFIG         = 26;
	const uint8_t GYRO_CONFIG    = 27;
	const uint8_t ACCEL_CONFIG   = 28;
	const uint8_t ACCEL_CONFIG_2 = 29;
	const uint8_t LP_ACCEL_ODR   = 30;
	const uint8_t FIFO_EN        = 35;
	const uint8_t INT_PIN_CFG    = 55;
	const uint8_t INT_ENABLE     = 56;
	const uint8_t INT_STATUS     = 58;
	const uint8_t ACCEL_XOUT_H   = 59;
	const uint8_t USER_CTRL      = 106;
	const uint8_t PWR_MGMT_1     = 107;
	const uint8_t PWR_MGMT_2     = 108;
	const uint8_t WHO_AM_I       = 117;
	//-------------------------------------------
	uint8_t SMPLRT_DIV_value     = 0;
	uint8_t CONFIG_value         = 0;
	uint8_t GYRO_CONFIG_value    = 0;
	uint8_t ACCEL_CONFIG_value   = 0;
	uint8_t ACCEL_CONFIG_2_value = 0;
	uint8_t LP_ACCEL_ODR_value   = 0;
	uint8_t FIFO_EN_value        = 0;
	uint8_t INT_PIN_CFG_value    = 0;
	uint8_t INT_ENABLE_value     = 0;
	uint8_t INT_STATUS_value     = 0;
	uint8_t ACCEL_XOUT_H_value   = 0;
	uint8_t USER_CTRL_value      = 0;
	uint8_t PWR_MGMT_1_value     = 0;
	uint8_t PWR_MGMT_2_value     = 0;
	uint8_t WHO_AM_I_value       = 0;
};

#endif /* MAIN_MPU9250_H_ */
