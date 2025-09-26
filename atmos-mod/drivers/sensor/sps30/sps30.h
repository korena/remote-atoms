#ifndef ZEPHYR_DRIVERS_SENSOR_SPS30_H_
#define ZEPHYR_DRIVERS_SENSOR_SPS30_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define SCD41_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

enum sensor_channel_ext {
  SENSOR_CHAN_PM_MC_1p0=68,
  SENSOR_CHAN_PM_MC_2p5,
  SENSOR_CHAN_PM_MC_4p0,
  SENSOR_CHAN_PM_MC_10p0,
  SENSOR_CHAN_PM_NC_0p5,
  SENSOR_CHAN_PM_NC_1p0,
  SENSOR_CHAN_PM_NC_2p5,
  SENSOR_CHAN_PM_NC_4p0,
  SENSOR_CHAN_PM_NC_10p0,
  SENSOR_CHAN_PM_TPS,
};

#endif
