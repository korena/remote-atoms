#ifndef ZEPHYR_DRIVERS_SENSOR_SPS30_H_
#define ZEPHYR_DRIVERS_SENSOR_SPS30_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define SCD41_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#define SCD41_I2C_ADDR_62 (0x69)

typedef int (*scd41_bus_check_fn)(const struct device *dev);
typedef int (*scd41_reg_read_fn)(const struct device *dev,
				  uint8_t start, uint8_t *buf, int size);
typedef int (*scd41_reg_write_fn)(const struct device *dev,
				   uint8_t reg, uint8_t val);


struct scd41_bus_io {
	scd41_bus_check_fn check;
	scd41_reg_read_fn read;
	scd41_reg_write_fn write;
};

typedef enum {
    SPS30_START_MEASUREMENT_CMD_ID = 0x10,
    SPS30_STOP_MEASUREMENT_CMD_ID = 0x104,
    SPS30_READ_DATA_READY_FLAG_CMD_ID = 0x202,
    SPS30_READ_MEASUREMENT_VALUES_UINT16_CMD_ID = 0x300,
    SPS30_READ_MEASUREMENT_VALUES_FLOAT_CMD_ID = 0x300,
    SPS30_SLEEP_CMD_ID = 0x1001,
    SPS30_WAKE_UP_CMD_ID = 0x1103,
    SPS30_START_FAN_CLEANING_CMD_ID = 0x5607,
    SPS30_READ_AUTO_CLEANING_INTERVAL_CMD_ID = 0x8004,
    SPS30_WRITE_AUTO_CLEANING_INTERVAL_CMD_ID = 0x8004,
    SPS30_READ_PRODUCT_TYPE_CMD_ID = 0xd002,
    SPS30_READ_SERIAL_NUMBER_CMD_ID = 0xd033,
    SPS30_READ_FIRMWARE_VERSION_CMD_ID = 0xd100,
    SPS30_READ_DEVICE_STATUS_REGISTER_CMD_ID = 0xd206,
    SPS30_CLEAR_DEVICE_STATUS_REGISTER_CMD_ID = 0xd210,
    SPS30_DEVICE_RESET_CMD_ID = 0xd304,
} SPS30_CMD_ID;

typedef enum {
    SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_FLOAT = 768,
    SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_UINT16 = 1280,
} sps30_output_format;

#endif
