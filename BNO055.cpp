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
BNO055::BNO055(I2CDevice::i2c_t* i2c, axis_config_t axis_config) :
	i2c(i2c, i2c_addr, Struct::lsb_first),
	acc_x(&(this->i2c), reg_acc_x_addr),
	acc_y(&(this->i2c), reg_acc_y_addr),
	acc_z(&(this->i2c), reg_acc_z_addr),
	mag_x(&(this->i2c), reg_mag_x_addr),
	mag_y(&(this->i2c), reg_mag_y_addr),
	mag_z(&(this->i2c), reg_mag_z_addr),
	gyr_x(&(this->i2c), reg_gyr_x_addr),
	gyr_y(&(this->i2c), reg_gyr_y_addr),
	gyr_z(&(this->i2c), reg_gyr_z_addr),
	eul_h(&(this->i2c), reg_eul_h_addr),
	eul_r(&(this->i2c), reg_eul_r_addr),
	eul_p(&(this->i2c), reg_eul_p_addr),
	qua_w(&(this->i2c), reg_qua_w_addr),
	qua_x(&(this->i2c), reg_qua_x_addr),
	qua_y(&(this->i2c), reg_qua_y_addr),
	qua_z(&(this->i2c), reg_qua_z_addr),
	lia_x(&(this->i2c), reg_lia_x_addr),
	lia_y(&(this->i2c), reg_lia_y_addr),
	lia_z(&(this->i2c), reg_lia_z_addr),
	grv_x(&(this->i2c), reg_grv_x_addr),
	grv_y(&(this->i2c), reg_grv_y_addr),
	grv_z(&(this->i2c), reg_grv_z_addr)
{
	this->axis_config = axis_config;
}

/**
 * @brief Initializes IMU and verifies I2C communication
 * @return Boolean indicating success of startup
 */
bool BNO055::init()
{
	// Set to Configuration Mode
	i2c.set(reg_opmode_addr, reg_opmode_config);

	// Verify chip data
	i2c.get_seq(reg_id_addr, 4);
	bool test1 = ((uint8_t)i2c == reg_id_chip);
	bool test2 = ((uint8_t)i2c == reg_id_acc);
	bool test3 = ((uint8_t)i2c == reg_id_mag);
	bool test4 = ((uint8_t)i2c == reg_id_gyr);
	bool connected = test1 && test2 && test3 && test4;

	// Set IMU to 9-DOF mode and configure axis-remapping
	if (connected)
	{
		i2c.set(reg_pwrmode_addr, reg_pwrmode_norm);
		uint8_t reg_axis_cfg_val = 0x00;
		uint8_t reg_axis_sgn_val = 0x00;
		switch (axis_config)
		{
			case NWU: reg_axis_cfg_val = 0x24; reg_axis_sgn_val = 0x00; break;
			case ENU: reg_axis_cfg_val = 0x21; reg_axis_sgn_val = 0x04; break;
			case NED: reg_axis_cfg_val = 0x24; reg_axis_sgn_val = 0x03; break;
			case WND: reg_axis_cfg_val = 0x21; reg_axis_sgn_val = 0x01; break;
		}
		i2c.set(reg_axis_config_addr, reg_axis_cfg_val);
		i2c.set(reg_axis_sign_addr, reg_axis_sgn_val);
		i2c.set(reg_opmode_addr, reg_opmode_ndof);
	}

	// Return connection status
	return connected;
}

/**
 * @brief Returns true if system is calibrated
 */
bool BNO055::calibrated()
{
	uint8_t reg_cal = (uint8_t)i2c.get_seq(reg_cal_addr, 1);
	return (reg_cal & reg_cal_mask) == reg_cal_mask;
}

/**
 * @brief Updates all IMU readings
 */
void BNO055::update()
{
	i2c.get_seq(reg_acc_x_addr, 32);
	acc_x.update(); acc_y.update(); acc_z.update();
	mag_x.update(); mag_y.update(); mag_z.update();
	gyr_x.update(); gyr_y.update(); gyr_z.update();
	eul_h.update(); eul_r.update(); eul_p.update();
	qua_w.update(); qua_x.update(); qua_y.update(); qua_z.update();
	i2c.get_seq(reg_lia_x_addr, 12);
	lia_x.update(); lia_y.update(); lia_z.update();
	grv_x.update(); grv_y.update(); grv_z.update();
}

/**
 * @brief Updates accelerometer readings
 */
void BNO055::update_acc()
{
	i2c.get_seq(reg_acc_x_addr, 6);
	acc_x.update();
	acc_y.update();
	acc_z.update();
}

/**
 * @brief Returns X acceleration [m/s^2]
 */
float BNO055::get_acc_x()
{
	return acc_x * acc_per_lsb;
}

/**
 * @brief Returns Y acceleration [m/s^2]
 */
float BNO055::get_acc_y()
{
	return acc_y * acc_per_lsb;
}

/**
 * @brief Returns Z acceleration [m/s^2]
 */
float BNO055::get_acc_z()
{
	return acc_z * acc_per_lsb;
}

/**
 * @brief Updates magnetometer readings
 */
void BNO055::update_mag()
{
	i2c.get_seq(reg_mag_x_addr, 6);
	mag_x.update();
	mag_y.update();
	mag_z.update();
}

/**
 * @brief Returns X magnetic field [uT]
 */
float BNO055::get_mag_x()
{
	return mag_x * mag_per_lsb;
}

/**
 * @brief Returns Y magnetic field [uT]
 */
float BNO055::get_mag_y()
{
	return mag_y * mag_per_lsb;
}

/**
 * @brief Returns Z magnetic field [uT]
 */
float BNO055::get_mag_z()
{
	return mag_z * mag_per_lsb;
}

/**
 * @brief Updates gyroscope readings
 */
void BNO055::update_gyr()
{
	i2c.get_seq(reg_gyr_x_addr, 6);
	gyr_x.update();
	gyr_y.update();
	gyr_z.update();
}

/**
 * @brief Returns X angular velocity [rad/s]
 */
float BNO055::get_gyr_x()
{
	return gyr_x * gyr_per_lsb;
}

/**
 * @brief Returns Y angular velocity [rad/s]
 */
float BNO055::get_gyr_y()
{
	return gyr_y * gyr_per_lsb;
}

/**
 * @brief Returns Z angular velocity [rad/s]
 */
float BNO055::get_gyr_z()
{
	return gyr_z * gyr_per_lsb;
}

/**
 * @brief Updates euler angle readings
 */
void BNO055::update_eul()
{
	i2c.get_seq(reg_eul_h_addr, 6);
	eul_h.update();
	eul_r.update();
	eul_p.update();
}

/**
 * @brief Returns euler heading [rad]
 */
float BNO055::get_eul_h()
{
	return eul_h * eul_per_lsb;
}

/**
 * @brief Returns euler roll [rad]
 */
float BNO055::get_eul_r()
{
	return eul_r * eul_per_lsb;
}

/**
 * @brief Returns euler pitch [rad]
 */
float BNO055::get_eul_p()
{
	return eul_p * eul_per_lsb;
}

/**
 * @brief Updates quaternion readings
 */
void BNO055::update_qua()
{
	i2c.get_seq(reg_qua_w_addr, 8);
	qua_w.update();
	qua_x.update();
	qua_y.update();
	qua_z.update();
}

/**
 * @brief Returns quaternion W
 */
float BNO055::get_qua_w()
{
	return qua_w * qua_per_lsb;
}

/**
 * @brief Returns quaternion X
 */
float BNO055::get_qua_x()
{
	return qua_x * qua_per_lsb;
}

/**
 * @brief Returns quaternion Y
 */
float BNO055::get_qua_y()
{
	return qua_y * qua_per_lsb;
}

/**
 * @brief Returns quaternion Z
 */
float BNO055::get_qua_z()
{
	return qua_z * qua_per_lsb;
}

/**
 * @brief Updates linear acceleration readings
 */
void BNO055::update_lia()
{
	i2c.get_seq(reg_lia_x_addr, 6);
	lia_x.update();
	lia_y.update();
	lia_z.update();
}

/**
 * @brief Returns X linear acceleration [m/s^2]
 */
float BNO055::get_lia_x()
{
	return lia_x * acc_per_lsb;
}

/**
 * @brief Returns Y linear acceleration [m/s^2]
 */
float BNO055::get_lia_y()
{
	return lia_y * acc_per_lsb;
}

/**
 * @brief Returns Z linear acceleration [m/s^2]
 */
float BNO055::get_lia_z()
{
	return lia_z * acc_per_lsb;
}

/**
 * @brief Updates gravity vector readings
 */
void BNO055::update_grv()
{
	i2c.get_seq(reg_grv_x_addr, 6);
	grv_x.update();
	grv_y.update();
	grv_z.update();
}

/**
 * @brief Returns gravity vector X [m/s^2]
 */
float BNO055::get_grv_x()
{
	return grv_x * acc_per_lsb;
}

/**
 * @brief Returns gravity vector Y [m/s^2]
 */
float BNO055::get_grv_y()
{
	return grv_y * acc_per_lsb;
}

/**
 * @brief Returns gravity vector Z [m/s^2]
 */
float BNO055::get_grv_z()
{
	return grv_z * acc_per_lsb;
}