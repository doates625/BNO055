/**
 * @file BNO055.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "BNO055.h"

/**
 * Static Constants
 */
const float BNO055::acc_per_cnt = 0.01000000000;
const float BNO055::gyr_per_cnt = 0.00111111111;
const float BNO055::eul_per_cnt = 0.00109170305;

/**
 * @brief Constructs BNO055 interface
 * @param wire TwoWire I2C interface
 * @param axis_config Axis remapping (see header)
 */
BNO055::BNO055(I2CDEVICE_I2C_CLASS* i2c, axis_config_t axis_config)
{
	this->i2c = I2CDevice(i2c, i2c_addr, I2CDevice::lsb_first);
	this->axis_config = axis_config;
}

/**
 * @brief Initializes IMU and verifies I2C communication
 * @return Boolean indicating success of startup
 */
bool BNO055::init()
{
	// Set to Configuration Mode
	i2c.write_uint8(reg_opmode_addr, reg_opmode_config);

	// Verify chip data
	i2c.read_sequence(reg_id_addr, 4);
	bool test1 = (i2c.read_uint8() == reg_id_chip);
	bool test2 = (i2c.read_uint8() == reg_id_acc);
	bool test3 = (i2c.read_uint8() == reg_id_mag);
	bool test4 = (i2c.read_uint8() == reg_id_gyr);
	bool connected = test1 && test2 && test3 && test4;

	// Set IMU to 9-DOF mode and configure axis-remapping
	if (connected)
	{
		i2c.write_uint8(reg_pwrmode_addr, reg_pwrmode_norm);
		uint8_t reg_axis_cfg_val = 0x00;
		uint8_t reg_axis_sgn_val = 0x00;
		switch (axis_config)
		{
			case tlf: reg_axis_cfg_val = 0x21; reg_axis_sgn_val = 0x04; break;
			case trf: reg_axis_cfg_val = 0x24; reg_axis_sgn_val = 0x00; break;
			case tlb: reg_axis_cfg_val = 0x24; reg_axis_sgn_val = 0x06; break;
			case trb: reg_axis_cfg_val = 0x21; reg_axis_sgn_val = 0x02; break;
			case dlf: reg_axis_cfg_val = 0x24; reg_axis_sgn_val = 0x03; break;
			case drf: reg_axis_cfg_val = 0x21; reg_axis_sgn_val = 0x01; break;
			case dlb: reg_axis_cfg_val = 0x21; reg_axis_sgn_val = 0x07; break;
			case drb: reg_axis_cfg_val = 0x24; reg_axis_sgn_val = 0x05; break;
		}
		i2c.write_uint8(reg_axis_config_addr, reg_axis_cfg_val);
		i2c.write_uint8(reg_axis_sign_addr, reg_axis_sgn_val);
		i2c.write_uint8(reg_opmode_addr, reg_opmode_ndof);
	}

	// Return connection status
	return connected;
}

/**
 * @brief Reads accelerometer, gyro, and euler angles from device
 */
void BNO055::update()
{
	// Read Accelerometer
	i2c.read_sequence(reg_acc_x_addr, 6);
	acc_x = i2c.read_int16() * acc_per_cnt;
	acc_y = i2c.read_int16() * acc_per_cnt;
	acc_z = i2c.read_int16() * acc_per_cnt;

	// Read Gyroscope
	i2c.read_sequence(reg_gyro_x_addr, 6);
	vel_x = i2c.read_int16() * gyr_per_cnt;
	vel_y = i2c.read_int16() * gyr_per_cnt;
	vel_z = i2c.read_int16() * gyr_per_cnt;

	// Read Euler Angles
	i2c.read_sequence(reg_euler_h_addr, 6);
	eul_h = i2c.read_int16() * eul_per_cnt;
	eul_r = i2c.read_int16() * eul_per_cnt;
	eul_p = i2c.read_int16() * eul_per_cnt;
}

/**
 * @brief Returns x-velocity [rad/s]
 */
float BNO055::get_vel_x()
{
	return vel_x;
}

/**
 * @brief Returns y-velocity [rad/s]
 */
float BNO055::get_vel_y()
{
	return vel_y;
}

/**
 * @brief Returns z-velocity [rad/s]
 */
float BNO055::get_vel_z()
{
	return vel_z;
}

/**
 * @brief returns x-acceleration [m/s^2]
 */
float BNO055::get_acc_x()
{
	return acc_x;
}

/**
 * @brief returns y-acceleration [m/s^2]
 */
float BNO055::get_acc_y()
{
	return acc_y;
}

/**
 * @brief returns z-acceleration [m/s^2]
 */
float BNO055::get_acc_z()
{
	return acc_z;
}

/**
 * @brief Returns euler heading [rad]
 */
float BNO055::get_eul_h()
{
	return eul_h;
}

/**
 * @brief Returns euler pitch [rad]
 */
float BNO055::get_eul_p()
{
	return eul_p;
}

/**
 * @brief Returns euler roll [rad]
 */
float BNO055::get_eul_r()
{
	return eul_r;
}