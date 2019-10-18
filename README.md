# BNO055
Class for interfacing with the BNO055 I2C IMU  
Written by Dan Oates (WPI Class of 2020)

### Description
The BNO055 9-axis IMU computes absolute orientation, angular velocity, and gravity-adjusted acceleration. This class acts as an I2C interface with the device for both the Arduino and Mbed platforms.  
  
The BNO055 supports axis remapping. The enumeration axis_config_t refers to the position of the dot on the chip relative to the forward-facing orientation of the body. For example:
- tlf = Top, Left, Front
- drb = Down, Right, Back

### Dependencies
- [Unions](https://github.com/doates625/Unions.git)
- [Platform](https://github.com/doates625/Platform.git)
- [I2CDevice](https://github.com/doates625/I2CDevice.git)

### References
- [Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)