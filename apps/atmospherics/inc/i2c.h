#ifndef _ATMOS_I2C_H_
#define _ATMOS_I2C_H_

#include <zephyr/device.h>

int i2c_devices_detect(const struct device* i2c_controller);

#endif // _ATMOS_I2C_H_
