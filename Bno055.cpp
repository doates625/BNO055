/**
 * @file BNO055.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "BNO055.h"

/**
 * Static Constants
 */
const float BNO055::acc_per_lsb = 1.0f / 100.0f;
const float BNO055::mag_per_lsb = 1.0f / 16.0f;
const float BNO055::gyr_per_lsb = 1.0f / 900.0f;
const float BNO055::eul_per_lsb = 1.0f / 900.0f;
const float BNO055::qua_per_lsb = 1.0f / 16384.0f;

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
 * @brief Updates all IMU readings
 */
void BNO055::update()
{
#if defined(PLATFORM_ARDUINO)
	i2c.read_sequence(reg_acc_x_addr, 32);
	read_acc();
	read_mag();
	read_gyr();
	read_eul();
	read_qua();
	i2c.read_sequence(reg_lia_x_addr, 12);
	read_lia();
	read_grv();
#elif defined(PLATFORM_MBED)
	i2c.read_sequence(reg_acc_x_addr, 44);
	read_acc();
	read_mag();
	read_gyr();
	read_eul();
	read_qua();
	read_lia();
	read_grv();
#endif
}

/**
 * @brief Updates accelerometer readings
 */
void BNO055::update_acc()
{
	i2c.read_sequence(reg_acc_x_addr, 6);
	read_acc();
}

/**
 * @brief Returns X acceleration [m/s^2]
 */
float BNO055::get_acc_x()
{
	return read_acc_x ? acc_x : i2c.read_int16(reg_acc_x_addr) * acc_per_lsb;
}

/**
 * @brief Returns Y acceleration [m/s^2]
 */
float BNO055::get_acc_y()
{
	return read_acc_y ? acc_y : i2c.read_int16(reg_acc_y_addr) * acc_per_lsb;
}

/**
 * @brief Returns Z acceleration [m/s^2]
 */
float BNO055::get_acc_z()
{
	return read_acc_z ? acc_z : i2c.read_int16(reg_acc_z_addr) * acc_per_lsb;
}

// Magnetometer

/**
 * @brief Updates magnetometer readings
 */
void BNO055::update_mag()
{
	i2c.read_sequence(reg_mag_x_addr, 6);
	read_mag();
}

/**
 * @brief Returns X magnetic field [uT]
 */
float BNO055::get_mag_x()
{
	return read_mag_x ? mag_x : i2c.read_int16(reg_mag_x_addr) * mag_per_lsb;
}

/**
 * @brief Returns Y magnetic field [uT]
 */
float BNO055::get_mag_y()
{
	return read_mag_y ? mag_y : i2c.read_int16(reg_mag_y_addr) * mag_per_lsb;
}

/**
 * @brief Returns Z magnetic field [uT]
 */
float BNO055::get_mag_z()
{
	return read_mag_z ? mag_z : i2c.read_int16(reg_mag_z_addr) * mag_per_lsb;
}

/**
 * @brief Updates gyroscope readings
 */
void BNO055::update_gyr()
{
	i2c.read_sequence(reg_gyr_x_addr, 6);
	read_gyr();
}

/**
 * @brief Returns X angular velocity [rad/s]
 */
float BNO055::get_gyr_x()
{
	return read_gyr_x ? gyr_x : i2c.read_int16(reg_gyr_x_addr) * gyr_per_lsb;
}

/**
 * @brief Returns Y angular velocity [rad/s]
 */
float BNO055::get_gyr_y()
{
	return read_gyr_y ? gyr_y : i2c.read_int16(reg_gyr_y_addr) * gyr_per_lsb;
}

/**
 * @brief Returns Z angular velocity [rad/s]
 */
float BNO055::get_gyr_z()
{
	return read_gyr_z ? gyr_z : i2c.read_int16(reg_gyr_z_addr) * gyr_per_lsb;
}

/**
 * @brief Updates euler angle readings
 */
void BNO055::update_eul()
{
	i2c.read_sequence(reg_eul_h_addr, 6);
	read_eul();
}

/**
 * @brief Returns euler heading [rad]
 */
float BNO055::get_eul_h()
{
	return read_eul_h ? eul_h : i2c.read_int16(reg_eul_h_addr) * eul_per_lsb;
}

/**
 * @brief Returns euler roll [rad]
 */
float BNO055::get_eul_r()
{
	return read_eul_r ? eul_r : i2c.read_int16(reg_eul_r_addr) * eul_per_lsb;
}

/**
 * @brief Returns euler pitch [rad]
 */
float BNO055::get_eul_p()
{
	return read_eul_p ? eul_p : i2c.read_int16(reg_eul_p_addr) * eul_per_lsb;
}

/**
 * @brief Updates quaternion readings
 */
void BNO055::update_qua()
{
	i2c.read_sequence(reg_qua_x_addr, 8);
	read_qua();
}

/**
 * @brief Returns quaternion W
 */
float BNO055::get_qua_w()
{
	return read_qua_w ? qua_w : i2c.read_int16(reg_qua_w_addr) * qua_per_lsb;
}

/**
 * @brief Returns quaternion X
 */
float BNO055::get_qua_x()
{
	return read_qua_x ? qua_x : i2c.read_int16(reg_qua_x_addr) * qua_per_lsb;
}

/**
 * @brief Returns quaternion Y
 */
float BNO055::get_qua_y()
{
	return read_qua_y ? qua_y : i2c.read_int16(reg_qua_y_addr) * qua_per_lsb;
}

/**
 * @brief Returns quaternion Z
 */
float BNO055::get_qua_z()
{
	return read_qua_z ? qua_z : i2c.read_int16(reg_qua_z_addr) * qua_per_lsb;
}

/**
 * @brief Updates linear acceleration readings
 */
void BNO055::update_lia()
{
	i2c.read_sequence(reg_lia_x_addr, 6);
	read_lia();
}

/**
 * @brief Returns X linear acceleration [m/s^2]
 */
float BNO055::get_lia_x()
{
	return read_lia_x ? lia_x : i2c.read_int16(reg_lia_x_addr) * acc_per_lsb;
}

/**
 * @brief Returns Y linear acceleration [m/s^2]
 */
float BNO055::get_lia_y()
{
	return read_lia_y ? lia_y : i2c.read_int16(reg_lia_y_addr) * acc_per_lsb;
}

/**
 * @brief Returns Z linear acceleration [m/s^2]
 */
float BNO055::get_lia_z()
{
	return read_lia_z ? lia_z : i2c.read_int16(reg_lia_z_addr) * acc_per_lsb;
}

/**
 * @brief Updates gravity vector readings
 */
void BNO055::update_grv()
{
	i2c.read_sequence(reg_grv_x_addr, 6);
	read_grv();
}

/**
 * @brief Returns gravity vector X [m/s^2]
 */
float BNO055::get_grv_x()
{
	return read_grv_x ? grv_x : i2c.read_int16(reg_grv_x_addr) * acc_per_lsb;
}

/**
 * @brief Returns gravity vector Y [m/s^2]
 */
float BNO055::get_grv_y()
{
	return read_grv_y ? grv_y : i2c.read_int16(reg_grv_y_addr) * acc_per_lsb;
}

/**
 * @brief Returns gravity vector Z [m/s^2]
 */
float BNO055::get_grv_z()
{
	return read_grv_z ? grv_z : i2c.read_int16(reg_grv_z_addr) * acc_per_lsb;
}

/**
 * @brief Reads acc registers after call to I2CDevice::read_sequence()
 */
void BNO055::read_acc()
{
	acc_x = i2c.read_int16() * acc_per_lsb; read_acc_x = true;
	acc_y = i2c.read_int16() * acc_per_lsb; read_acc_y = true;
	acc_z = i2c.read_int16() * acc_per_lsb; read_acc_z = true;
}

/**
 * @brief Reads mag registers after call to I2CDevice::read_sequence()
 */
void BNO055::read_mag()
{
	mag_x = i2c.read_int16() * mag_per_lsb; read_mag_x = true;
	mag_y = i2c.read_int16() * mag_per_lsb; read_mag_y = true;
	mag_z = i2c.read_int16() * mag_per_lsb; read_mag_z = true;
}

/**
 * @brief Reads gyr registers after call to I2CDevice::read_sequence()
 */
void BNO055::read_gyr()
{
	gyr_x = i2c.read_int16() * gyr_per_lsb; read_gyr_x = true;
	gyr_y = i2c.read_int16() * gyr_per_lsb; read_gyr_y = true;
	gyr_z = i2c.read_int16() * gyr_per_lsb; read_gyr_z = true;
}

/**
 * @brief Reads eul registers after call to I2CDevice::read_sequence()
 */
void BNO055::read_eul()
{
	eul_h = i2c.read_int16() * eul_per_lsb; read_eul_h = true;
	eul_r = i2c.read_int16() * eul_per_lsb; read_eul_r = true;
	eul_p = i2c.read_int16() * eul_per_lsb; read_eul_p = true;
}

/**
 * @brief Reads qua registers after call to I2CDevice::read_sequence()
 */
void BNO055::read_qua()
{
	qua_w = i2c.read_int16() * qua_per_lsb; read_qua_w = true;
	qua_x = i2c.read_int16() * qua_per_lsb; read_qua_x = true;
	qua_y = i2c.read_int16() * qua_per_lsb; read_qua_y = true;
	qua_z = i2c.read_int16() * qua_per_lsb; read_qua_z = true;
}

/**
 * @brief Reads lia registers after call to I2CDevice::read_sequence()
 */
void BNO055::read_lia()
{
	lia_x = i2c.read_int16() * acc_per_lsb; read_lia_x = true;
	lia_y = i2c.read_int16() * acc_per_lsb; read_lia_y = true;
	lia_z = i2c.read_int16() * acc_per_lsb; read_lia_z = true;
}

/**
 * @brief Reads grv registers after call to I2CDevice::read_sequence()
 */
void BNO055::read_grv()
{
	grv_x = i2c.read_int16() * acc_per_lsb; read_grv_x = true;
	grv_y = i2c.read_int16() * acc_per_lsb; read_grv_y = true;
	grv_z = i2c.read_int16() * acc_per_lsb; read_grv_z = true;
}