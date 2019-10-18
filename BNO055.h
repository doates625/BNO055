/**
 * @file BNO055.h
 * @brief Class for interfacing with the BNO055 I2C IMU
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <I2CDevice.h>

/**
 * I2CDevice Macro Update
 */
#if defined(PLATFORM_MBED) && I2CDEVICE_BUFFER_SIZE < 44
	#warning BNO055 requires I2CDEVICE_BUFFER_SIZE >= 44. Setting to 44...
	#undef I2CDEVICE_BUFFER_SIZE
	#define I2CDEVICE_BUFFER_SIZE 44
#endif

/**
 * Class Declaration
 */
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

	// Constructor and Basics
	BNO055(I2CDEVICE_I2C_CLASS* i2c, axis_config_t axis_config = tlf);
	bool init();
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
	float acc_x; bool read_acc_x = false;
	float acc_y; bool read_acc_y;
	float acc_z; bool read_acc_z;
	void read_acc();

	// Magnetometer
	static const float mag_per_lsb;
	static const uint8_t reg_mag_x_addr = 0x0E;
	static const uint8_t reg_mag_y_addr = 0x10;
	static const uint8_t reg_mag_z_addr = 0x12;
	float mag_x; bool read_mag_x;
	float mag_y; bool read_mag_y;
	float mag_z; bool read_mag_z;
	void read_mag();

	// Gyroscope
	static const float gyr_per_lsb;
	static const uint8_t reg_gyr_x_addr = 0x14;
	static const uint8_t reg_gyr_y_addr = 0x16;
	static const uint8_t reg_gyr_z_addr = 0x18;
	float gyr_x; bool read_gyr_x;
	float gyr_y; bool read_gyr_y;
	float gyr_z; bool read_gyr_z;
	void read_gyr();

	// I2C Euler Angle Registers
	static const float eul_per_lsb;
	static const uint8_t reg_eul_h_addr = 0x1A;
	static const uint8_t reg_eul_r_addr = 0x1C;
	static const uint8_t reg_eul_p_addr = 0x1E;
	float eul_h; bool read_eul_h;
	float eul_r; bool read_eul_r;
	float eul_p; bool read_eul_p;
	void read_eul();

	// I2C Quaternion Registers
	static const float qua_per_lsb;
	static const uint8_t reg_qua_w_addr = 0x20;
	static const uint8_t reg_qua_x_addr = 0x22;
	static const uint8_t reg_qua_y_addr = 0x24;
	static const uint8_t reg_qua_z_addr = 0x26;
	float qua_w; bool read_qua_w;
	float qua_x; bool read_qua_x;
	float qua_y; bool read_qua_y;
	float qua_z; bool read_qua_z;
	void read_qua();

	// I2C Linear Acceleration Registers
	static const uint8_t reg_lia_x_addr = 0x28;
	static const uint8_t reg_lia_y_addr = 0x2A;
	static const uint8_t reg_lia_z_addr = 0x2C;
	float lia_x; bool read_lia_x;
	float lia_y; bool read_lia_y;
	float lia_z; bool read_lia_z;
	void read_lia();

	// I2C Gravity Vector Registers
	static const uint8_t reg_grv_x_addr = 0x2E;
	static const uint8_t reg_grv_y_addr = 0x30;
	static const uint8_t reg_grv_z_addr = 0x32;
	float grv_x; bool read_grv_x;
	float grv_y; bool read_grv_y;
	float grv_z; bool read_grv_z;
	void read_grv();
};