/*
 * LSM9DS1.h
 *
 *  Created on: Apr 4, 2021
 *      Author: gilg
 */

#ifndef MAIN_LSM9DS1_H_
#define MAIN_LSM9DS1_H_

#include <stdint.h>
#include "SPIbus.h"
#include "Eigen/Core"

class LSM9DS1 {
public:
	LSM9DS1(spi_host_device_t spi_bus, int mosi_pin, int miso_pin, int sck_pin,int cs_pin_accel_gyro, int cs_pin_mag);
	~LSM9DS1();
	void read_raw_data();
	void read_data();
	void software_reset();
	int init();
//	LSM9DS1(const LSM9DS1 &other);
//	LSM9DS1(LSM9DS1 &&other);
//	LSM9DS1& operator=(const LSM9DS1 &other);
//	LSM9DS1& operator=(LSM9DS1 &&other);
private:
    spi_device_handle_t device_accel_gyro = 0;
    spi_device_handle_t device_mag = 0;
    SPI spi_bus;
    uint8_t RAW_GYRO[7];
    uint8_t RAW_ACCEL[7];
    uint8_t RAW_MAG[7];
    int16_t TEMPERATURE_LSB = 0;
    int16_t GYRO_LSB [3];
    int16_t ACCEL_LSB [3];
    int16_t MAG_LSB [3];
    Eigen::Vector3f angular_vel_degree;
    Eigen::Vector3f acceleration_mg;
    Eigen::Vector3f magnetic_field;

private:
    /////////////////////////////////////////
    // LSM9DS1 Accel/Gyro (XL/G) Registers //
    /////////////////////////////////////////
    const uint8_t ACT_THS           = 0x04;
    const uint8_t ACT_DUR           = 0x05;
    const uint8_t INT_GEN_CFG_XL    = 0x06;
    const uint8_t INT_GEN_THS_X_XL  = 0x07;
    const uint8_t INT_GEN_THS_Y_XL  = 0x08;
    const uint8_t INT_GEN_THS_Z_XL  = 0x09;
    const uint8_t INT_GEN_DUR_XL    = 0x0A;
    const uint8_t REFERENCE_G       = 0x0B;
    const uint8_t INT1_CTRL         = 0x0C;
    const uint8_t INT2_CTRL         = 0x0D;
    const uint8_t WHO_AM_I_XG       = 0x0F;
    const uint8_t CTRL_REG1_G       = 0x10;
    const uint8_t CTRL_REG2_G       = 0x11;
    const uint8_t CTRL_REG3_G       = 0x12;
    const uint8_t ORIENT_CFG_G      = 0x13;
    const uint8_t INT_GEN_SRC_G     = 0x14;
    const uint8_t OUT_TEMP_L        = 0x15;
    const uint8_t OUT_TEMP_H        = 0x16;
    const uint8_t STATUS_REG_0      = 0x17;
    const uint8_t OUT_X_L_G	        = 0x18;
    const uint8_t OUT_X_H_G	        = 0x19;
    const uint8_t OUT_Y_L_G	        = 0x1A;
    const uint8_t OUT_Y_H_G	        = 0x1B;
    const uint8_t OUT_Z_L_G	        = 0x1C;
    const uint8_t OUT_Z_H_G	        = 0x1D;
    const uint8_t CTRL_REG4	        = 0x1E;
    const uint8_t CTRL_REG5_XL      = 0x1F;
    const uint8_t CTRL_REG6_XL      = 0x20;
    const uint8_t CTRL_REG7_XL      = 0x21;
    const uint8_t CTRL_REG8	 = 0x22;
    const uint8_t CTRL_REG9	 = 0x23;
    const uint8_t CTRL_REG10	 = 0x24;
    const uint8_t INT_GEN_SRC_XL = 0x26;
    const uint8_t STATUS_REG_1 = 0x27;
    const uint8_t OUT_X_L_XL	 = 0x28;
    const uint8_t OUT_X_H_XL	 = 0x29;
    const uint8_t OUT_Y_L_XL	 = 0x2A;
    const uint8_t OUT_Y_H_XL	 = 0x2B;
    const uint8_t OUT_Z_L_XL	 = 0x2C;
    const uint8_t OUT_Z_H_XL	 = 0x2D;
    const uint8_t FIFO_CTRL	 = 0x2E;
    const uint8_t FIFO_SRC	 = 0x2F;
    const uint8_t INT_GEN_CFG_G = 0x30;
    const uint8_t INT_GEN_THS_XH_G = 0x31;
    const uint8_t INT_GEN_THS_XL_G = 0x32;
    const uint8_t INT_GEN_THS_YH_G = 0x33;
    const uint8_t INT_GEN_THS_YL_G = 0x34;
    const uint8_t INT_GEN_THS_ZH_G = 0x35;
    const uint8_t INT_GEN_THS_ZL_G = 0x36;
    const uint8_t INT_GEN_DUR_G = 0x37;

    ///////////////////////////////
    // LSM9DS1 Magneto Registers //
    ///////////////////////////////
    const uint8_t OFFSET_X_REG_L_M = 0x05;
    const uint8_t OFFSET_X_REG_H_M = 0x06;
    const uint8_t OFFSET_Y_REG_L_M = 0x07;
    const uint8_t OFFSET_Y_REG_H_M = 0x08;
    const uint8_t OFFSET_Z_REG_L_M = 0x09;
    const uint8_t OFFSET_Z_REG_H_M = 0x0A;
    const uint8_t WHO_AM_I_M	 = 0x0F;
    const uint8_t CTRL_REG1_M	 = 0x20;
    const uint8_t CTRL_REG2_M	 = 0x21;
    const uint8_t CTRL_REG3_M	 = 0x22;
    const uint8_t CTRL_REG4_M	 = 0x23;
    const uint8_t CTRL_REG5_M	 = 0x24;
    const uint8_t STATUS_REG_M   = 0x27;
    const uint8_t OUT_X_L_M	     = 0x28;
    const uint8_t OUT_X_H_M	     = 0x29;
    const uint8_t OUT_Y_L_M	     = 0x2A;
    const uint8_t OUT_Y_H_M	     = 0x2B;
    const uint8_t OUT_Z_L_M	     = 0x2C;
    const uint8_t OUT_Z_H_M	     = 0x2D;
    const uint8_t INT_CFG_M	     = 0x30;
    const uint8_t INT_SRC_M	     = 0x31;
    const uint8_t INT_THS_L_M	 = 0x32;
    const uint8_t INT_THS_H_M	 = 0x33;

    ////////////////////////////////
    // LSM9DS1 WHO_AM_I Responses //
    ////////////////////////////////
    const uint8_t WHO_AM_I_AG_RSP = 0x68;
    const uint8_t WHO_AM_I_M_RSP = 0x3D;
private:
    float gyro_scalar_lsb_to_deg = 0;
    float accel_scalar_lsb_to_mg = 0;
    float magnetometr_lsb_to_uTesla = 0;
    Eigen::Matrix3f accel_gyro_to_mag_coord_system = (Eigen :: MatrixX3f(3,3) <<

     		-1, 0, 0,
     		0, -1, 0,
 			0, 0, 1).finished();
};

#endif /* MAIN_LSM9DS1_H_ */
