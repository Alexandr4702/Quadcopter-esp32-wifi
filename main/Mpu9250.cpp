/*
 * Mpu9250.cpp
 *
 *  Created on: Aug 6, 2020
 *      Author: alexandr
 */

#include "Mpu9250.h"

Mpu9250::Mpu9250() {

//	SPI_t &mySPI = vspi;  // vspi and hspi are the default objects

}

Mpu9250::~Mpu9250() {
}

void Mpu9250::init() {

	init_spi();
    uint8_t am = read_reg(WHO_AM_I);
    printf("%02hx \r\n",am);

    SMPLRT_DIV_value = 19;
    write_reg(SMPLRT_DIV, SMPLRT_DIV_value);

    uint8_t DLPF_CFG = 3;
    CONFIG_value = DLPF_CFG;
    write_reg(CONFIG, CONFIG_value);

    uint8_t Fchoice_b = 3;
    uint8_t GYRO_FS_SEL = 0x0;
    GYRO_CONFIG_value = Fchoice_b | GYRO_FS_SEL << 3;
    write_reg(GYRO_CONFIG, GYRO_CONFIG_value);

    uint8_t ACCEL_FS_SEL = 1;
    ACCEL_CONFIG_value = 0 | ACCEL_FS_SEL << 3;
    write_reg(ACCEL_CONFIG, ACCEL_CONFIG_value);

    uint8_t accel_fchoice_b = 1;
    uint8_t A_DLPFCFG = 3;
    ACCEL_CONFIG_2_value = A_DLPFCFG | accel_fchoice_b << 3;
    write_reg(ACCEL_CONFIG_2, ACCEL_CONFIG_2_value);

    PWR_MGMT_1_value = 0;
    write_reg(PWR_MGMT_1, PWR_MGMT_1_value);

    PWR_MGMT_2_value = 0;
    write_reg(PWR_MGMT_2, PWR_MGMT_2_value);


    lsb_to_mg_accel = 4000.0f / 32768.0f;
    lsb_to_dps_gyro = 250.0f / 32768.0f;

}

void Mpu9250::write_reg(uint8_t reg, uint8_t data) {
	mySPI.writeByte(device_accel_gyro, reg, data);
}

uint8_t Mpu9250::read_reg(uint8_t reg) {
	uint8_t rx;
	mySPI.readBytes(device_accel_gyro, reg, 1, &rx);
	return rx;
}

void Mpu9250::read_raw_data() {
	read_regs(rx_buffer, 14);
    raw_data_accel[0] = (rx_buffer[1]) | rx_buffer[0] << 8;
    raw_data_accel[1] = (rx_buffer[3]) | rx_buffer[2] << 8;
    raw_data_accel[2] = (rx_buffer[5]) | rx_buffer[4] << 8;

    raw_temp = (rx_buffer[7]) | rx_buffer[6] << 8;

    raw_data_gyro[0] = (rx_buffer[9]) | rx_buffer[8] << 8;
    raw_data_gyro[1] = (rx_buffer[11]) | rx_buffer[10] << 8;
    raw_data_gyro[2] = (rx_buffer[13]) | rx_buffer[12] << 8;

}

void Mpu9250::init_spi() {
    spi_bus_config_t config;
    config.mosi_io_num = 23;
    config.miso_io_num = 19;
    config.sclk_io_num = 18;
    config.quadwp_io_num = -1;  // -1 not used
    config.quadhd_io_num = -1;  // -1 not used
    config.max_transfer_sz = 1000;
    config.flags = SPICOMMON_BUSFLAG_MASTER;
    config.intr_flags = 0;
    spi_bus_initialize(VSPI_HOST, &config, 0);
	ESP_ERROR_CHECK( mySPI.addDevice(0, 8000000, 5, &device_accel_gyro));

    spi_device_interface_config_t dev_config;
    dev_config.command_bits = 0;
    dev_config.address_bits = 8;
    dev_config.dummy_bits = 0;
    dev_config.mode = 0;
    dev_config.duty_cycle_pos = 128;  // default 128 = 50%/50% duty
    dev_config.cs_ena_pretrans = 0;  // 0 not used
    dev_config.cs_ena_posttrans = 0;  // 0 not used
    dev_config.clock_speed_hz = 1000000;
    dev_config.spics_io_num = 5;
    dev_config.flags = 0;  // 0 not used
    dev_config.queue_size = 1;
    dev_config.pre_cb = NULL;
    dev_config.post_cb = NULL;
    ESP_ERROR_CHECK( spi_bus_add_device(VSPI_HOST, &dev_config, &device_accel_gyro));
}

void Mpu9250::delay(uint32_t delay_) {
    vTaskDelay(delay_ / portTICK_PERIOD_MS);
}

void Mpu9250::read_regs(uint8_t *data, uint16_t size) {
	spi_transaction_t transaction;
	transaction.flags = 0;
	transaction.cmd = 0;
	transaction.addr = 0x3B | SPIBUS_READ;
	transaction.length = size * 8;
	transaction.rxlength = size * 8;
	transaction.user = NULL;
	transaction.tx_buffer = NULL;
	transaction.rx_buffer = data;
	ESP_ERROR_CHECK( spi_device_transmit(device_accel_gyro, &transaction));
}

void Mpu9250::read_data() {
	read_raw_data();

	accel       = Eigen::Vector3f(static_cast<float>(raw_data_accel[0]), static_cast<float>(raw_data_accel[1]), static_cast<float>(raw_data_accel[2]));
	anguar_velo = Eigen::Vector3f(static_cast<float>(raw_data_gyro[0]), static_cast<float>(raw_data_gyro[1]), static_cast<float>(raw_data_gyro[2]));
	mag         = Eigen::Vector3f(static_cast<float>(raw_data_mag[0]), static_cast<float>(raw_data_mag[1]), static_cast<float>(raw_data_mag[2]));

	accel       *= lsb_to_mg_accel;
	anguar_velo *= lsb_to_dps_gyro;
	mag         *= lsb_to_nT_mag;
}
