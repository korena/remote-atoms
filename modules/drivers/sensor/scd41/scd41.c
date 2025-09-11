#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>

#include "scd41.h"
#include <errno.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(SCD41, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "SCD41 driver enabled without any devices"
#endif

struct scd41_data {
  uint16_t serial_number;
  scd4x_sensor_variant variant;
  uint16_t co2_concentration;
  uint16_t temperature;
  uint16_t relative_humidity;
};

struct scd41_config {
  struct i2c_dt_spec i2c;
  const struct scd41_bus_io *bus_io;
};

// sensirion ported defines
#define SENSIRION_WORD_SIZE (2)
#define NO_ERROR (0)
#define CRC8_POLYNOMIAL (0x31)
#define CRC8_INIT (0xFF)
#define CRC_ERROR (1)
#define CRC8_LEN (0)

// Sensor API
// ---------------------------------------------------------------------------------------------------
static int scd41_chip_init(const struct device *dev);
static int scd41_sample_fetch(const struct device *dev,
                              enum sensor_channel chan);
static int scd41_channel_get(const struct device *dev, enum sensor_channel chan,
                             struct sensor_value *val);
static inline int scd41_bus_check(const struct device *dev);
// Private functions
// --------------------------------------------------------------------------------------------
static inline int scd41_reg_read(const struct device *dev, uint8_t start,
                                 uint8_t *buf, int size);
static inline int scd41_reg_write(const struct device *dev, uint8_t reg,
                                  uint8_t val);
static uint16_t scd41_add_command16_to_buffer(uint8_t *buffer, uint16_t offset,
                                              uint16_t command);
static int16_t scd41_i2c_read_data_inplace(const struct device *dev,
                                           uint8_t *buffer_ptr,
                                           uint16_t expected_data_length);
static int8_t scd41_i2c_check_crc(const uint8_t *data, uint16_t count,
                                  uint8_t checksum);
static uint8_t scd41_i2c_generate_crc(const uint8_t *data, uint16_t count);
static uint16_t scd41_bytes_to_uint16_t(const uint8_t *bytes);
static void scd41_copy_bytes(const uint8_t *source, uint8_t *destination,
                             uint16_t data_length);
static int16_t scd41_get_serial_number(const struct device *dev,
                                       uint16_t *serial_number,
                                       uint16_t serial_number_size);
static int16_t scd4x_get_sensor_variant_raw(const struct device* dev,
                                            uint16_t* sensor_variant);
static int16_t scd4x_get_sensor_variant(const struct device* dev,
                                        scd4x_sensor_variant* a_sensor_variant);
const struct scd41_bus_io scd41_bus_io_i2c = {
    .check = scd41_bus_check,
    .read = scd41_reg_read,
    .write = scd41_reg_write,
};

static const struct sensor_driver_api scd41_api_funcs = {
    .sample_fetch = scd41_sample_fetch,
    .channel_get = scd41_channel_get,
};

static int scd41_chip_init(const struct device *dev) {
  struct scd41_data *data = dev->data;
  int err;

  err = scd41_bus_check(dev);
  if (err < 0) {
    LOG_DBG("bus check failed: %d", err);
    return err;
  }

  err = scd41_get_serial_number(dev, &data->serial_number, 3);
  if (err < 0) {
    LOG_DBG("ID read failed: %d", err);
    return err;
  }

  err = scd4x_get_sensor_variant(dev , &data->variant);

  LOG_DBG("SCD41(0x%x)", data->serial_number);

  if (data->variant != SCD4X_SENSOR_VARIANT_SCD41) {
      LOG_DBG("SCD4x variant not supported (0x%x)", data->variant);
      return -ENOTSUP;
  }
  LOG_DBG("SCD4x variant(0x%x)", data->variant);

  err = scd41_reg_write(dev, BME280_REG_RESET, BME280_CMD_SOFT_RESET);
  if (err < 0) {
    LOG_DBG("Soft-reset failed: %d", err);
  }

  err = bme280_wait_until_ready(dev);
  if (err < 0) {
    return err;
  }

  err = bme280_read_compensation(dev);
  if (err < 0) {
    return err;
  }

  if (data->chip_id == BME280_CHIP_ID) {
    err = bme280_reg_write(dev, BME280_REG_CTRL_HUM, BME280_HUMIDITY_OVER);
    if (err < 0) {
      LOG_DBG("CTRL_HUM write failed: %d", err);
      return err;
    }
  }

  err = bme280_reg_write(dev, BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_VAL);
  if (err < 0) {
    LOG_DBG("CTRL_MEAS write failed: %d", err);
    return err;
  }

  err = bme280_reg_write(dev, BME280_REG_CONFIG, BME280_CONFIG_VAL);
  if (err < 0) {
    LOG_DBG("CONFIG write failed: %d", err);
    return err;
  }
  /* Wait for the sensor to be ready */
  k_sleep(K_MSEC(1));

  LOG_DBG("\"%s\" OK", dev->name);
  return 0;
}

static int scd41_sample_fetch(const struct device *dev,
                              enum sensor_channel chan) {
  struct scd41_data *data = dev->data;
  uint8_t buf[16];
  uint16_t *co2_concentration, temperature, relative_humidity;
  int size = 6;
  int ret;

  __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
}
static int scd41_channel_get(const struct device *dev, enum sensor_channel chan,
                             struct sensor_value *val) {
  return 0;
}

static uint16_t scd41_bytes_to_uint16_t(const uint8_t *bytes) {
  return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

static void scd41_copy_bytes(const uint8_t *source, uint8_t *destination,
                             uint16_t data_length) {
  uint16_t i;
  for (i = 0; i < data_length; i++) {
    destination[i] = source[i];
  }
}

static uint16_t scd41_add_command16_to_buffer(uint8_t *buffer, uint16_t offset,
                                              uint16_t command) {
  buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
  buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
  return offset;
}

static uint8_t scd41_i2c_generate_crc(const uint8_t *data, uint16_t count) {
  uint16_t current_byte;
  uint8_t crc = CRC8_INIT;
  uint8_t crc_bit;

  /* calculates 8-Bit checksum with given polynomial */
  for (current_byte = 0; current_byte < count; ++current_byte) {
    crc ^= (data[current_byte]);
    for (crc_bit = 8; crc_bit > 0; --crc_bit) {
      if (crc & 0x80)
        crc = (crc << 1) ^ CRC8_POLYNOMIAL;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

static int8_t scd41_i2c_check_crc(const uint8_t *data, uint16_t count,
                                  uint8_t checksum) {
  if (scd41_i2c_generate_crc(data, count) != checksum)
    return CRC_ERROR;
  return NO_ERROR;
}

static int16_t scd41_i2c_read_data_inplace(const struct device *dev,
                                           uint8_t *buffer_ptr,
                                           uint16_t expected_data_length) {
  int16_t error;
  uint16_t i, j;
  uint16_t size = (expected_data_length / SENSIRION_WORD_SIZE) *
                  (SENSIRION_WORD_SIZE + CRC8_LEN);
  const struct scd41_config *cfg = dev->config;

  if (expected_data_length % SENSIRION_WORD_SIZE != 0) {
    return BYTE_NUM_ERROR;
  }

  error = i2c_read(cfg->i2c->bus, buffer_ptr, size, cfg->i2c->addr);
  if (error) {
    return error;
  }
  for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {

    error = scd41_i2c_check_crc(&buffer[i], SENSIRION_WORD_SIZE,
                                buffer[i + SENSIRION_WORD_SIZE]);
    if (error) {
      return error;
    }
    buffer[j++] = buffer[i];
    buffer[j++] = buffer[i + 1];
  }
  return NO_ERROR;
}

static int16_t scd41_get_data_ready_status_raw(const struct device *dev,
                                               uint16_t *data_ready_status) {
  int16_t local_error = NO_ERROR;
  uint8_t buffer_ptr[9];
  uint16_t local_offset = 0;
  const struct scd41_config *cfg = dev->config;
  local_offset = scd41_add_command16_to_buffer(
      buffer_ptr, local_offset, SCD4X_GET_DATA_READY_STATUS_RAW_CMD_ID);
  local_error = i2c_write(cfg->i2c->bus, buffer_ptr, local_offset, cfg->i2c->addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_usleep(K_MSEC(1));
  local_error = scd41_i2c_read_data_inplace(dev, buffer_ptr, 2);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *data_ready_status = scd41_bytes_to_uint16_t(&buffer_ptr[0]);
  return local_error;
}

static int16_t scd41_get_data_ready_status(const struct device *dev,
                                           bool *arg_0) {
  uint16_t data_ready_status = 0;
  int16_t local_error = 0;
  const struct scd41_config *cfg = dev->config;
  local_error = scd4x_get_data_ready_status_raw(&data_ready_status);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *arg_0 = (data_ready_status & 2047) != 0;
  return local_error;
}

static inline int scd41_bus_check(const struct device *dev) {
  const struct scd41_config *cfg = dev->config;
  return device_is_ready(cfg->i2c.bus) ? 0 : -ENODEV;
}

static inline int scd41_reg_read(const struct device *dev, uint8_t start,
                                 uint8_t *buf, int size) {
  const struct scd41_config *cfg = dev->config;
  return i2c_burst_read_dt(cfg->i2c, start, buf, size);
}

static inline int scd41_reg_write(const struct device *dev, uint8_t reg,
                                  uint8_t val) {
  const struct scd41_config *cfg = dev->config;
  return i2c_reg_write_byte_dt(cfg->i2c, reg, val);
}

static int16_t scd41_get_serial_number(const struct device *dev,
                                       uint16_t *serial_number,
                                       uint16_t serial_number_size) {
  int16_t local_error = NO_ERROR;
  uint8_t buffer_ptr[9];
  uint16_t local_offset = 0;
  const struct scd41_config *cfg = dev->config;
  local_offset = scd41_add_command16_to_buffer(buffer_ptr, local_offset,
                                               SCD4X_GET_SERIAL_NUMBER_CMD_ID);
  local_error =
      i2c_write(cfg->i2c->bus, buffer_ptr, local_offset, cfg->i2c->addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_usleep(K_MSEC(1));
  local_error = scd41_i2c_read_data_inplace(dev, buffer_ptr, 6);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  scd41_copy_bytes(&buffer_ptr[0], (uint8_t *)serial_number,
                   (serial_number_size * 2));
  return local_error;
}

static int16_t scd4x_get_sensor_variant(const struct device* dev, scd4x_sensor_variant* a_sensor_variant) {
    scd4x_sensor_variant ret_val = SCD4X_SENSOR_VARIANT_MASK;
    uint16_t raw_sensor_variant = 0;
    uint16_t my_sensor_variant = 0;
    int16_t local_error = 0;
    ret_val = SCD4X_SENSOR_VARIANT_MASK;
    uint16_t mask = (uint16_t)(ret_val);
    local_error = scd4x_get_sensor_variant_raw(&raw_sensor_variant);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    my_sensor_variant = (uint16_t)(raw_sensor_variant & mask);
    if (my_sensor_variant == (uint16_t)(SCD4X_SENSOR_VARIANT_SCD40)) {
        ret_val = SCD4X_SENSOR_VARIANT_SCD40;
    } else if (my_sensor_variant == (uint16_t)(SCD4X_SENSOR_VARIANT_SCD41)) {
        ret_val = SCD4X_SENSOR_VARIANT_SCD41;
    } else if (my_sensor_variant == (uint16_t)(SCD4X_SENSOR_VARIANT_SCD42)) {
        ret_val = SCD4X_SENSOR_VARIANT_SCD42;
    } else if (my_sensor_variant == (uint16_t)(SCD4X_SENSOR_VARIANT_SCD43)) {
        ret_val = SCD4X_SENSOR_VARIANT_SCD43;
    }
    *a_sensor_variant = ret_val;
    return local_error;
}

static int16_t scd4x_get_sensor_variant_raw(const struct device* dev, uint16_t* sensor_variant) {
    int16_t local_error = NO_ERROR;
    uint8_t buffer_ptr[9] = {0};
    uint16_t local_offset = 0;
    const struct scd41_config *cfg = dev->config;
    local_offset =
        scd41_add_command16_to_buffer(buffer_ptr, local_offset, SCD4X_GET_SENSOR_VARIANT_RAW_CMD_ID);
    local_error = i2c_write(cfg->i2c->bus, buffer_ptr, local_offset, cfg->i2c->addr);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    k_usleep(K_MSEC(1));
    local_error = scd41_i2c_read_data_inplace(dev, buffer_ptr, 2);
    if (local_error != NO_ERROR) {
        return local_error;
    }
    *sensor_variant = scd41_bytes_to_uint16_t(&buffer_ptr[0]);
    return local_error;
}

