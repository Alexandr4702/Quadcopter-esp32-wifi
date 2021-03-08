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
	uint8_t rx_buffer[40] = {0};

	float lsb_to_mg_accel = 4000.0f / 32768.0f;;
	float lsb_to_dps_gyro = 250.0f / 32768.0f;
	float lsb_to_nT_mag   = 1;
	float temp=0;

	Eigen::Vector3f accel;

	Eigen::Vector3f anguar_velo;
	Eigen::Vector3f anguar_velo_bias;

	Eigen::Vector3f mag;
	Eigen::Vector3f mag_gain_from_rom;
	Eigen::Matrix3f mag_soft;
	Eigen::Matrix3f mag_hard;
private:
	void write_reg (uint8_t reg, uint8_t data);
	void read_registers (uint8_t reg, uint8_t* data, uint8_t cnt);
	uint8_t read_reg (uint8_t reg);
	void read_raw_data ();
	void write_register_to_AK8963 (uint8_t reg,uint8_t data);
	void read_registers_to_AK8963 (uint8_t reg, uint8_t* data, uint8_t cnt);
	uint8_t read_reg_to_AK8963 (uint8_t reg);
	void delay(uint32_t delay_);
	void setup_i2c_AK8963();
	void reset_mpu();
	void setup_AK8963();
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
	const uint8_t SELF_TEST_X_GYRO =  0x00;
	const uint8_t SELF_TEST_Y_GYRO =  0x01;
	const uint8_t SELF_TEST_Z_GYRO =  0x02;
	const uint8_t SELF_TEST_X_ACCEL = 0x0D;
	const uint8_t SELF_TEST_Y_ACCEL = 0x0E;
	const uint8_t SELF_TEST_Z_ACCEL = 0x0F;
	const uint8_t XG_OFFSET_H =       0x13;
	const uint8_t XG_OFFSET_L =       0x14;
	const uint8_t YG_OFFSET_H =       0x15;
	const uint8_t YG_OFFSET_L =       0x16;
	const uint8_t ZG_OFFSET_H =       0x17;
	const uint8_t ZG_OFFSET_L =       0x18;
	const uint8_t SMPLRT_DIV =        0x19;
	const uint8_t CONFIG =            0x1A;
	const uint8_t GYRO_CONFIG =       0x1B;
	const uint8_t ACCEL_CONFIG =      0x1C;
	const uint8_t ACCEL_CONFIG_2 =    0x1D;
	const uint8_t LP_ACCEL_ODR =      0x1E;
	const uint8_t WOM_THR =           0x1F;
	const uint8_t FIFO_EN =           0x23;
	const uint8_t I2C_MST_CTRL =      0x24;
	const uint8_t I2C_SLV0_ADDR =     0x25;
	const uint8_t I2C_SLV0_REG =      0x26;
	const uint8_t I2C_SLV0_CTRL =     0x27;
	const uint8_t I2C_SLV1_ADDR =     0x28;
	const uint8_t I2C_SLV1_REG =      0x29;
	const uint8_t I2C_SLV1_CTRL =     0x2A;
	const uint8_t I2C_SLV2_ADDR =     0x2B;
	const uint8_t I2C_SLV2_REG =      0x2C;
	const uint8_t I2C_SLV2_CTRL =     0x2D;
	const uint8_t I2C_SLV3_ADDR =     0x2E;
	const uint8_t I2C_SLV3_REG =      0x2F;
	const uint8_t I2C_SLV3_CTRL =     0x30;
	const uint8_t I2C_SLV4_ADDR =     0x31;
	const uint8_t I2C_SLV4_REG =      0x32;
	const uint8_t I2C_SLV4_DO =       0x33;
	const uint8_t I2C_SLV4_CTRL =     0x34;
	const uint8_t I2C_SLV4_DI =       0x35;
	const uint8_t I2C_MST_STATUS =    0x36;
	const uint8_t INT_PIN_CFG =       0x37;
	const uint8_t INT_ENABLE =        0x38;
	const uint8_t INT_STATUS =        0x3A;
	const uint8_t ACCEL_XOUT_H =      0x3B;
	const uint8_t ACCEL_XOUT_L =      0x3C;
	const uint8_t ACCEL_YOUT_H =      0x3D;
	const uint8_t ACCEL_YOUT_L =      0x3E;
	const uint8_t ACCEL_ZOUT_H =      0x3F;
	const uint8_t ACCEL_ZOUT_L =      0x40;
	const uint8_t TEMP_OUT_H =        0x41;
	const uint8_t TEMP_OUT_L =        0x42;
	const uint8_t GYRO_XOUT_H =       0x43;
	const uint8_t GYRO_XOUT_L =       0x44;
	const uint8_t GYRO_YOUT_H =       0x45;
	const uint8_t GYRO_YOUT_L =       0x46;
	const uint8_t GYRO_ZOUT_H =       0x47;
	const uint8_t GYRO_ZOUT_L =       0x48;
	const uint8_t EXT_SENS_DATA_00 =  0x49;
	const uint8_t EXT_SENS_DATA_01 =  0x4A;
	const uint8_t EXT_SENS_DATA_02 =  0x4B;
	const uint8_t EXT_SENS_DATA_03 =  0x4C;
	const uint8_t EXT_SENS_DATA_04 =  0x4D;
	const uint8_t EXT_SENS_DATA_05 =  0x4E;
	const uint8_t EXT_SENS_DATA_06 =  0x4F;
	const uint8_t EXT_SENS_DATA_07 =  0x50;
	const uint8_t EXT_SENS_DATA_08 =  0x51;
	const uint8_t EXT_SENS_DATA_09 =  0x52;
	const uint8_t EXT_SENS_DATA_10 =  0x53;
	const uint8_t EXT_SENS_DATA_11 =  0x54;
	const uint8_t EXT_SENS_DATA_12 =  0x55;
	const uint8_t EXT_SENS_DATA_13 =  0x56;
	const uint8_t EXT_SENS_DATA_14 =  0x57;
	const uint8_t EXT_SENS_DATA_15 =  0x58;
	const uint8_t EXT_SENS_DATA_16 =  0x59;
	const uint8_t EXT_SENS_DATA_17 =  0x5A;
	const uint8_t EXT_SENS_DATA_18 =  0x5B;
	const uint8_t EXT_SENS_DATA_19 =  0x5C;
	const uint8_t EXT_SENS_DATA_20 =  0x5D;
	const uint8_t EXT_SENS_DATA_21 =  0x5E;
	const uint8_t EXT_SENS_DATA_22 =  0x5F;
	const uint8_t EXT_SENS_DATA_23 =  0x60;
	const uint8_t I2C_SLV0_DO =       0x63;
	const uint8_t I2C_SLV1_DO =       0x64;
	const uint8_t I2C_SLV2_DO =       0x65;
	const uint8_t I2C_SLV3_DO =       0x66;
	const uint8_t I2C_MST_DELAY_CTRL =0x67;
	const uint8_t SIGNAL_PATH_RESET = 0x68;
	const uint8_t MOT_DETECT_CTRL =   0x69;
	const uint8_t USER_CTRL =         0x6A;
	const uint8_t PWR_MGMT_1 =        0x6B;
	const uint8_t PWR_MGMT_2 =        0x6C;
	const uint8_t FIFO_COUNTH =       0x72;
	const uint8_t FIFO_COUNTL =       0x73;
	const uint8_t FIFO_R_W =          0x74;
	const uint8_t WHO_AM_I =          0x75;
	const uint8_t XA_OFFSET_H =       0x77;
	const uint8_t XA_OFFSET_L =       0x78;
	const uint8_t YA_OFFSET_H =       0x7A;
	const uint8_t YA_OFFSET_L =       0x7B;
	const uint8_t ZA_OFFSET_H =       0x7D;
	const uint8_t ZA_OFFSET_L =       0x7E;
	//-------------------------------------------
	uint8_t SELF_TEST_X_GYRO_value =  0x00;
	uint8_t SELF_TEST_Y_GYRO_value =  0x01;
	uint8_t SELF_TEST_Z_GYRO_value =  0x02;
	uint8_t SELF_TEST_X_ACCEL_value = 0x0D;
	uint8_t SELF_TEST_Y_ACCEL_value = 0x0E;
	uint8_t SELF_TEST_Z_ACCEL_value = 0x0F;
	uint8_t XG_OFFSET_H_value =       0x13;
	uint8_t XG_OFFSET_L_value =       0x14;
	uint8_t YG_OFFSET_H_value =       0x15;
	uint8_t YG_OFFSET_L_value =       0x16;
	uint8_t ZG_OFFSET_H_value =       0x17;
	uint8_t ZG_OFFSET_L_value =       0x18;
	uint8_t SMPLRT_DIV_value =        0x19;
	uint8_t CONFIG_value =            0x1A;
	uint8_t GYRO_CONFIG_value =       0x1B;
	uint8_t ACCEL_CONFIG_value =      0x1C;
	uint8_t ACCEL_CONFIG_2_value =    0x1D;
	uint8_t LP_ACCEL_ODR_value =      0x1E;
	uint8_t WOM_THR_value =           0x1F;
	uint8_t FIFO_EN_value =           0x23;
	uint8_t I2C_MST_CTRL_value =      0x24;
	uint8_t I2C_SLV0_ADDR_value =     0x25;
	uint8_t I2C_SLV0_REG_value =      0x26;
	uint8_t I2C_SLV0_CTRL_value =     0x27;
	uint8_t I2C_SLV1_ADDR_value =     0x28;
	uint8_t I2C_SLV1_REG_value =      0x29;
	uint8_t I2C_SLV1_CTRL_value =     0x2A;
	uint8_t I2C_SLV2_ADDR_value =     0x2B;
	uint8_t I2C_SLV2_REG_value =      0x2C;
	uint8_t I2C_SLV2_CTRL_value =     0x2D;
	uint8_t I2C_SLV3_ADDR_value =     0x2E;
	uint8_t I2C_SLV3_REG_value =      0x2F;
	uint8_t I2C_SLV3_CTRL_value =     0x30;
	uint8_t I2C_SLV4_ADDR_value =     0x31;
	uint8_t I2C_SLV4_REG_value =      0x32;
	uint8_t I2C_SLV4_DO_value =       0x33;
	uint8_t I2C_SLV4_CTRL_value =     0x34;
	uint8_t I2C_SLV4_DI_value =       0x35;
	uint8_t I2C_MST_STATUS_value =    0x36;
	uint8_t INT_PIN_CFG_value =       0x37;
	uint8_t INT_ENABLE_value =        0x38;
	uint8_t INT_STATUS_value =        0x3A;
	uint8_t ACCEL_XOUT_H_value =      0x3B;
	uint8_t ACCEL_XOUT_L_value =      0x3C;
	uint8_t ACCEL_YOUT_H_value =      0x3D;
	uint8_t ACCEL_YOUT_L_value =      0x3E;
	uint8_t ACCEL_ZOUT_H_value =      0x3F;
	uint8_t ACCEL_ZOUT_L_value =      0x40;
	uint8_t TEMP_OUT_H_value =        0x41;
	uint8_t TEMP_OUT_L_value =        0x42;
	uint8_t GYRO_XOUT_H_value =       0x43;
	uint8_t GYRO_XOUT_L_value =       0x44;
	uint8_t GYRO_YOUT_H_value =       0x45;
	uint8_t GYRO_YOUT_L_value =       0x46;
	uint8_t GYRO_ZOUT_H_value =       0x47;
	uint8_t GYRO_ZOUT_L_value =       0x48;
	uint8_t EXT_SENS_DATA_00_value =  0x49;
	uint8_t EXT_SENS_DATA_01_value =  0x4A;
	uint8_t EXT_SENS_DATA_02_value =  0x4B;
	uint8_t EXT_SENS_DATA_03_value =  0x4C;
	uint8_t EXT_SENS_DATA_04_value =  0x4D;
	uint8_t EXT_SENS_DATA_05_value =  0x4E;
	uint8_t EXT_SENS_DATA_06_value =  0x4F;
	uint8_t EXT_SENS_DATA_07_value =  0x50;
	uint8_t EXT_SENS_DATA_08_value =  0x51;
	uint8_t EXT_SENS_DATA_09_value =  0x52;
	uint8_t EXT_SENS_DATA_10_value =  0x53;
	uint8_t EXT_SENS_DATA_11_value =  0x54;
	uint8_t EXT_SENS_DATA_12_value =  0x55;
	uint8_t EXT_SENS_DATA_13_value =  0x56;
	uint8_t EXT_SENS_DATA_14_value =  0x57;
	uint8_t EXT_SENS_DATA_15_value =  0x58;
	uint8_t EXT_SENS_DATA_16_value =  0x59;
	uint8_t EXT_SENS_DATA_17_value =  0x5A;
	uint8_t EXT_SENS_DATA_18_value =  0x5B;
	uint8_t EXT_SENS_DATA_19_value =  0x5C;
	uint8_t EXT_SENS_DATA_20_value =  0x5D;
	uint8_t EXT_SENS_DATA_21_value =  0x5E;
	uint8_t EXT_SENS_DATA_22_value =  0x5F;
	uint8_t EXT_SENS_DATA_23_value =  0x60;
	uint8_t I2C_SLV0_DO_value =       0x63;
	uint8_t I2C_SLV1_DO_value =       0x64;
	uint8_t I2C_SLV2_DO_value =       0x65;
	uint8_t I2C_SLV3_DO_value =       0x66;
	uint8_t I2C_MST_DELAY_CTRL_value =0x67;
	uint8_t SIGNAL_PATH_RESET_value = 0x68;
	uint8_t MOT_DETECT_CTRL_value =   0x69;
	uint8_t USER_CTRL_value =         0x6A;
	uint8_t PWR_MGMT_1_value =        0x6B;
	uint8_t PWR_MGMT_2_value =        0x6C;
	uint8_t FIFO_COUNTH_value =       0x72;
	uint8_t FIFO_COUNTL_value =       0x73;
	uint8_t FIFO_R_W_value =          0x74;
	uint8_t WHO_AM_I_value =          0x75;
	uint8_t XA_OFFSET_H_value =       0x77;
	uint8_t XA_OFFSET_L_value =       0x78;
	uint8_t YA_OFFSET_H_value =       0x7A;
	uint8_t YA_OFFSET_L_value =       0x7B;
	uint8_t ZA_OFFSET_H_value =       0x7D;
	uint8_t ZA_OFFSET_L_value =       0x7E;
private:
	// AK8963 addres
	const uint8_t AK8963_I2C_ADDR = 0x0C;
	// AK8963 registers
	const uint8_t WIA    = 0;
	const uint8_t INFO   = 1;
	const uint8_t ST1    = 2;
	const uint8_t HXL    = 3;
	const uint8_t HXH    = 4;
	const uint8_t HYL    = 5;
	const uint8_t HYH    = 6;
	const uint8_t HZL    = 7;
	const uint8_t HZH    = 8;
	const uint8_t ST2    = 9;
	const uint8_t CNTL1  = 10;
	const uint8_t CNTL2  = 11;
	const uint8_t ASTC   = 12;
	const uint8_t TS1    = 13;
	const uint8_t TS2    = 14;
	const uint8_t I2CDIS = 15;
	const uint8_t ASAX   = 16;
	const uint8_t ASAY   = 17;
	const uint8_t ASAZ   = 18;

	uint8_t WIA_value    = 0;
	uint8_t INFO_value   = 0;
	uint8_t ST1_value    = 0;
	uint8_t HXL_value    = 0;
	uint8_t HXH_value    = 0;
	uint8_t HYL_value    = 0;
	uint8_t HYH_value    = 0;
	uint8_t HZL_value    = 0;
	uint8_t HZH_value    = 0;
	uint8_t ST2_value    = 0;
	uint8_t CNTL1_value  = 0;
	uint8_t CNTL2_value  = 0;
	uint8_t ASTC_value   = 0;
	uint8_t TS1_value    = 0;
	uint8_t TS2_value    = 0;
	uint8_t I2CDIS_value = 0;
	uint8_t ASAX_value   = 0;
	uint8_t ASAY_value   = 0;
	uint8_t ASAZ_value   = 0;
};

#endif /* MAIN_MPU9250_H_ */
