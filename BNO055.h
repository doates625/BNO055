/**
 * @file BNO055.h
 * @brief Class for interfacing with the BNO055 I2C IMU
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <I2CDevice.h>
#include <I2CReading.h>

/**
 * I2CDevice Macro Update
 */
#if I2CDEVICE_BUFFER_SIZE != 32
	#error BNO055 requires I2CDEVICE_BUFFER_SIZE == 32
#endif

/**
 * Class Declaration
 */
class BNO055
{
public:

	// Axis config enum (x, y, z = ...)
	typedef enum
	{
		NWU,	// North West Up
		ENU,	// East North Up
		NED,	// North East Down
		WND,	// West North Down
	}
	axis_config_t;

	// Constructor and Basics
	BNO055(I2CDevice::i2c_t* i2c, axis_config_t axis_config = NWU);
	bool init();
	bool calibrated();
	void update();

	// Accelerometer
	void update_acc();
	float get_acc_x();
	float get_acc_y();
	float get_acc_z();

	// Magnetometer
	void update_mag();
	float get_mag_x();
	float get_mag_y();
	float get_mag_z();

	// Gyroscope
	void update_gyr();
	float get_gyr_x();
	float get_gyr_y();
	float get_gyr_z();

	// Euler Angles
	void update_eul();
	float get_eul_h();
	float get_eul_r();
	float get_eul_p();

	// Quaternion
	void update_qua();
	float get_qua_w();
	float get_qua_x();
	float get_qua_y();
	float get_qua_z();

	// Linear Acceleration
	void update_lia();
	float get_lia_x();
	float get_lia_y();
	float get_lia_z();

	// Gravity Vector
	void update_grv();
	float get_grv_x();
	float get_grv_y();
	float get_grv_z();

protected:

	// I2C Communication
	static const uint8_t i2c_addr = 0x28;
	I2CDevice i2c;

	// I2C ID Registers
	static const uint8_t reg_id_addr = 0x00;
	static const uint8_t reg_id_chip = 0xA0;
	static const uint8_t reg_id_acc = 0xFB;
	static const uint8_t reg_id_mag = 0x32;
	static const uint8_t reg_id_gyr = 0x0F;

	// Calibration Register
	static const uint8_t reg_cal_addr = 0x35;
	static const uint8_t reg_cal_mask = 0b11000000;

	// I2C Config Registers
	static const uint8_t reg_pwrmode_addr = 0x3E;
	static const uint8_t reg_pwrmode_norm = 0x00;
	static const uint8_t reg_opmode_addr = 0x3D;
	static const uint8_t reg_opmode_config = 0x00;
	static const uint8_t reg_opmode_ndof = 0x0C;
	static const uint8_t reg_axis_config_addr = 0x41;
	static const uint8_t reg_axis_sign_addr = 0x42;
	axis_config_t axis_config;

	// Accelerometer
	static const float acc_per_lsb;
	static const uint8_t reg_acc_x_addr = 0x08;
	static const uint8_t reg_acc_y_addr = 0x0A;
	static const uint8_t reg_acc_z_addr = 0x0C;
	I2CReading<int16_t> acc_x, acc_y, acc_z;

	// Magnetometer
	static const float mag_per_lsb;
	static const uint8_t reg_mag_x_addr = 0x0E;
	static const uint8_t reg_mag_y_addr = 0x10;
	static const uint8_t reg_mag_z_addr = 0x12;
	I2CReading<int16_t> mag_x, mag_y, mag_z;

	// Gyroscope
	static const float gyr_per_lsb;
	static const uint8_t reg_gyr_x_addr = 0x14;
	static const uint8_t reg_gyr_y_addr = 0x16;
	static const uint8_t reg_gyr_z_addr = 0x18;
	I2CReading<int16_t> gyr_x, gyr_y, gyr_z;

	// I2C Euler Angle Registers
	static const float eul_per_lsb;
	static const uint8_t reg_eul_h_addr = 0x1A;
	static const uint8_t reg_eul_r_addr = 0x1C;
	static const uint8_t reg_eul_p_addr = 0x1E;
	I2CReading<int16_t> eul_h, eul_r, eul_p;

	// I2C Quaternion Registers
	static const float qua_per_lsb;
	static const uint8_t reg_qua_w_addr = 0x20;
	static const uint8_t reg_qua_x_addr = 0x22;
	static const uint8_t reg_qua_y_addr = 0x24;
	static const uint8_t reg_qua_z_addr = 0x26;
	I2CReading<int16_t> qua_w, qua_x, qua_y, qua_z;

	// I2C Linear Acceleration Registers
	static const uint8_t reg_lia_x_addr = 0x28;
	static const uint8_t reg_lia_y_addr = 0x2A;
	static const uint8_t reg_lia_z_addr = 0x2C;
	I2CReading<int16_t> lia_x, lia_y, lia_z;

	// I2C Gravity Vector Registers
	static const uint8_t reg_grv_x_addr = 0x2E;
	static const uint8_t reg_grv_y_addr = 0x30;
	static const uint8_t reg_grv_z_addr = 0x32;
	I2CReading<int16_t> grv_x, grv_y, grv_z;
};