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

class Mpu9250 {
public:
	Mpu9250();
	virtual ~Mpu9250();
	void init();
	void write_reg(uint8_t ret,uint8_t data);

};

#endif /* MAIN_MPU9250_H_ */
