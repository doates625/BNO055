/**
 * @file BNO055.h
 * @brief Class for interfacing with BNO055 IMU
 * @author Dan Oates (WPI Class of 2020)
 * 
 * The BNO055 0-axis IMU computes absolute orientation, angular velocity, and
 * gravity-adjusted acceleration. This class acts as an I2C interface with the
 * device for both the Arduino and Mbed platforms.
 * 
 * The euler angle convertions are as follows:
 * - Heading increases modulo 2pi clockwise
 * - Positive pitch tilts the top of the body forwards
 * - Positive roll tilts the top of the body right
 * 
 * The BNO055 supports axis remapping. The enumeration axis_config_t refers to
 * the position of the dot on the chip relative to the forward-facing
 * orientationof the body. For example:
 * 
 * - tlf = Top, Left, Front
 * - drb = Down, Right, Back
 * 
 * Dependencies:
 * - I2CDevice: https://github.com/doates625/I2CDevice.git
 * - Platform: https://github.com/doates625/Platform.git
 * - Unions: https://github.com/doates625/Unions.git
 * 
 * References:
 * - Datasheet: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
 */
#pragma once
#include <I2CDevice.h>

class BNO055
{
public:

	// Axis config enum
	typedef enum
	{
		tlf,	// Top Left Front
		trf,	// Top Right Front
		tlb,	// Top Left Back
		trb,	// Top Right Back
		dlf,	// Down Left Front
		drf,	// Down Right Front
		dlb,	// Down Left Back
		drb,	// Down Right Back
	}
	axis_config_t;

	// Constructor and Initialization
	BNO055(I2CDEVICE_I2C_CLASS* i2c, axis_config_t axis_config = tlf);
	bool init();

	// Data Reading
	void update();
	float get_vel_x();
	float get_vel_y();
	float get_vel_z();
	float get_acc_x();
	float get_acc_y();
	float get_acc_z();
	float get_eul_h();
	float get_eul_p();
	float get_eul_r();

private:

	// I2C Registers
	static const uint8_t i2c_addr = 0x28;
	static const uint8_t reg_id_addr = 0x00;
	static const uint8_t reg_id_chip = 0xA0;
	static const uint8_t reg_id_acc = 0xFB;
	static const uint8_t reg_id_mag = 0x32;
	static const uint8_t reg_id_gyr = 0x0F;
	static const uint8_t reg_pwrmode_addr = 0x3E;
	static const uint8_t reg_pwrmode_norm = 0x00;
	static const uint8_t reg_opmode_addr = 0x3D;
	static const uint8_t reg_opmode_config = 0x00;
	static const uint8_t reg_opmode_ndof = 0x0C;
	static const uint8_t reg_axis_config_addr = 0x41;
	static const uint8_t reg_axis_sign_addr = 0x42;
	static const uint8_t reg_acc_x_addr = 0x28;
	static const uint8_t reg_acc_y_addr = 0x2A;
	static const uint8_t reg_acc_z_addr = 0x2C;
	static const uint8_t reg_euler_h_addr = 0x1A;
	static const uint8_t reg_euler_r_addr = 0x1C;
	static const uint8_t reg_euler_p_addr = 0x1E;
	static const uint8_t reg_gyro_x_addr = 0x14;
	static const uint8_t reg_gyro_y_addr = 0x16;
	static const uint8_t reg_gyro_z_addr = 0x18;

	// Unit Conversions
	static const float acc_per_cnt;
	static const float gyr_per_cnt;
	static const float eul_per_cnt;

	// Class Members
	I2CDevice i2c;
	axis_config_t axis_config;
	float vel_x, vel_y, vel_z;
	float acc_x, acc_y, acc_z;
	float eul_h, eul_p, eul_r;
};