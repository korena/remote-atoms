#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/types.h>

#include "sps30.h"
#include <zephyr/logging/log.h>

// sensirion ported defines
#define SENSIRION_WORD_SIZE (2)
#define NO_ERROR (0)
#define CRC8_POLYNOMIAL (0x31)
#define CRC8_INIT (0xFF)
#define CRC_ERROR (1)
#define I2C_BUS_ERROR (2)
#define I2C_NACK_ERROR (3)
#define BYTE_NUM_ERROR (4)
#define CRC8_LEN (1)

LOG_MODULE_REGISTER(SPS30, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT sensirion_sps30

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "SPS30 driver enabled without any devices"
#endif

typedef int (*sps30_bus_check_fn)(const struct device *dev);
typedef int (*sps30_reg_read_fn)(const struct device *dev, uint8_t start,
                                 uint8_t *buf, int size);
typedef int (*sps30_reg_write_fn)(const struct device *dev, uint8_t reg,
                                  uint8_t val);

struct sps30_data {
  uint8_t communication_buffer[60];
  uint16_t serial_number;
  uint16_t tps;
  uint16_t pm0_5;
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm4_0;
  uint16_t pm10_0;
};

struct sps30_config {
  struct i2c_dt_spec spec;
  const struct sps30_bus_io *bus_io;
  sps30_output_format measurement_output_format;
};

struct sps30_bus_io {
  sps30_bus_check_fn check;
  sps30_reg_read_fn read;
  sps30_reg_write_fn write;
};

// Sensor basic API
// ---------------------------------------------------------------------------------------------------
static int16_t sps30_read_product_type(const struct device *dev,
                                       int8_t *product_type,
                                       uint16_t product_type_size);
static int16_t sps30_read_serial_number(const struct device *dev,
                                        int8_t *serial_number,
                                        uint16_t serial_number_size);
static int16_t sps30_device_reset(const struct device *dev);
static int16_t sps30_read_firmware_version(const struct device *dev,
                                           uint8_t *major_version,
                                           uint8_t *minor_version);
static int16_t
sps30_read_auto_cleaning_interval(const struct device *dev,
                                  uint32_t *auto_cleaning_interval);
static int16_t sps30_read_device_status_register(const struct device *dev,
                                                 uint32_t *device_status);
static int16_t sps30_clear_device_status_register(const struct device *dev);
static int16_t sps30_sleep(const struct device *dev);
static int16_t sps30_start_fan_cleaning(const struct device *dev);
static int16_t
sps30_write_auto_cleaning_interval(const struct device *dev,
                                   uint32_t auto_cleaning_interval);
static int16_t sps30_wake_up(const struct device *dev);
static int16_t sps30_wake_up_sequence(const struct device *dev);
static int16_t sps30_start_measurement(const struct device *dev);
static int16_t sps30_stop_measurement(const struct device *dev);
static int16_t sps30_read_data_ready_flag(const struct device *dev,
                                          uint16_t *data_ready_flag);
static int16_t sps30_read_measurement_values_uint16(
    const struct device *dev, uint16_t *mc_1p0, uint16_t *mc_2p5,
    uint16_t *mc_4p0, uint16_t *mc_10p0, uint16_t *nc_0p5, uint16_t *nc_1p0,
    uint16_t *nc_2p5, uint16_t *nc_4p0, uint16_t *nc_10p0,
    uint16_t *typical_particle_size);

// device APIs
int sps30_chip_init(const struct device *dev);
static inline int sps30_bus_check(const struct device *dev);
static inline int sps30_reg_read(const struct device *dev, uint8_t start,
                                 uint8_t *buf, int size);
static inline int sps30_reg_write(const struct device *dev, uint8_t reg,
                                  uint8_t val);

static int sps30_sample_fetch(const struct device *dev,
                              enum sensor_channel chan);
static int sps30_channel_get(const struct device *dev, enum sensor_channel chan,
                             struct sensor_value *val);

static int sps30_attr_set(const struct device *dev, enum sensor_channel chan,
                          enum sensor_attribute attr,
                          const struct sensor_value *val);

static const struct sensor_driver_api sensor_sps30_api = {
    .sample_fetch = sps30_sample_fetch,
    .channel_get = sps30_channel_get,
    .attr_set = sps30_attr_set,
};
// private APIs
// ---------------------------------------------------------------------------------------------------
static uint16_t sps30_add_command16_to_buffer(uint8_t *buffer, uint16_t offset,
                                              uint16_t command);
static int16_t sps30_i2c_read_data_inplace(const struct device *dev,
                                           uint8_t *buffer_ptr,
                                           uint16_t expected_data_length);
static int8_t sps30_i2c_check_crc(const uint8_t *data, uint16_t count,
                                  uint8_t checksum);
static uint8_t sps30_i2c_generate_crc(const uint8_t *data, uint16_t count);
// Implementation
// ---------------------------------------------------------------------------------------------------

const struct sps30_bus_io sps30_bus_io_i2c = {
    .check = sps30_bus_check,
    .read = sps30_reg_read,
    .write = sps30_reg_write,
};

static uint8_t sps30_i2c_generate_crc(const uint8_t *data, uint16_t count) {
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

static int8_t sps30_i2c_check_crc(const uint8_t *data, uint16_t count,
                                  uint8_t checksum) {
  uint8_t generated = sps30_i2c_generate_crc(data, count);
  if (generated != checksum) {
    LOG_ERR("Checksum failed: checksum 0x%x, generated: 0x%x", checksum,
            generated);
    return CRC_ERROR;
  }
  return NO_ERROR;
}

static int16_t sps30_i2c_read_data_inplace(const struct device *dev,
                                           uint8_t *buffer_ptr,
                                           uint16_t expected_data_length) {
  int16_t error;
  uint16_t i, j;
  uint16_t size = (expected_data_length / SENSIRION_WORD_SIZE) *
                  (SENSIRION_WORD_SIZE + CRC8_LEN);
  const struct sps30_config *cfg = dev->config;

  if (expected_data_length % SENSIRION_WORD_SIZE != 0) {
    return BYTE_NUM_ERROR;
  }

  error = i2c_read(cfg->spec.bus, buffer_ptr, size, cfg->spec.addr);
  if (error) {
    LOG_ERR("Failed to read sensor (%d)", error);
    return error;
  }
  for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {
    error = sps30_i2c_check_crc(&buffer_ptr[i], SENSIRION_WORD_SIZE,
                                buffer_ptr[i + SENSIRION_WORD_SIZE]);
    if (error) {
      LOG_ERR("Failed crc (%d)", error);
      return error;
    }
    buffer_ptr[j++] = buffer_ptr[i];
    buffer_ptr[j++] = buffer_ptr[i + 1];
  }
  return NO_ERROR;
}

static void sps30_copy_bytes(const uint8_t *source, uint8_t *destination,
                             uint16_t data_length) {
  uint16_t i;
  for (i = 0; i < data_length; i++) {
    destination[i] = source[i];
  }
}

static uint16_t sensirion_common_bytes_to_uint16_t(const uint8_t *bytes) {
  return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

static uint32_t sensirion_common_bytes_to_uint32_t(const uint8_t *bytes) {
  return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
         (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

static int16_t sensirion_common_bytes_to_int16_t(const uint8_t *bytes) {
  return (int16_t)sensirion_common_bytes_to_uint16_t(bytes);
}

static int32_t sensirion_common_bytes_to_int32_t(const uint8_t *bytes) {
  return (int32_t)sensirion_common_bytes_to_uint32_t(bytes);
}

static uint16_t sps30_add_command16_to_buffer(uint8_t *buffer, uint16_t offset,
                                              uint16_t command) {
  buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
  buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
  return offset;
}

static uint16_t sensirion_i2c_add_uint16_t_to_buffer(uint8_t *buffer,
                                                     uint16_t offset,
                                                     uint16_t data) {
  buffer[offset++] = (uint8_t)((data & 0xFF00) >> 8);
  buffer[offset++] = (uint8_t)((data & 0x00FF) >> 0);
  buffer[offset] = sps30_i2c_generate_crc(&buffer[offset - SENSIRION_WORD_SIZE],
                                          SENSIRION_WORD_SIZE);
  offset++;

  return offset;
}

uint16_t sensirion_i2c_add_uint32_t_to_buffer(uint8_t *buffer, uint16_t offset,
                                              uint32_t data) {
  buffer[offset++] = (uint8_t)((data & 0xFF000000) >> 24);
  buffer[offset++] = (uint8_t)((data & 0x00FF0000) >> 16);
  buffer[offset] = sps30_i2c_generate_crc(&buffer[offset - SENSIRION_WORD_SIZE],
                                          SENSIRION_WORD_SIZE);
  offset++;
  buffer[offset++] = (uint8_t)((data & 0x0000FF00) >> 8);
  buffer[offset++] = (uint8_t)((data & 0x000000FF) >> 0);
  buffer[offset] = sps30_i2c_generate_crc(&buffer[offset - SENSIRION_WORD_SIZE],
                                          SENSIRION_WORD_SIZE);
  offset++;

  return offset;
}

int sps30_chip_init(const struct device *dev) {
  int err;
  err = sps30_bus_check(dev);
  if (err < 0) {
    LOG_DBG("bus check failed %d", err);
    return err;
  }

  return 0;
}
static inline int sps30_bus_check(const struct device *dev) {
  const struct sps30_config *cfg = dev->config;
  return device_is_ready(cfg->spec.bus) ? 0 : -ENODEV;
}

static inline int sps30_reg_read(const struct device *dev, uint8_t start,
                                 uint8_t *buf, int size) {
  const struct sps30_config *cfg = dev->config;
  return i2c_burst_read_dt(&cfg->spec, start, buf, size);
}

static inline int sps30_reg_write(const struct device *dev, uint8_t reg,
                                  uint8_t val) {
  const struct sps30_config *cfg = dev->config;
  return i2c_reg_write_byte_dt(&cfg->spec, reg, val);
}


static int sps30_sample_fetch(const struct device *dev,
                              enum sensor_channel chan) {
  return 0;
}


static int sps30_channel_get(const struct device *dev, enum sensor_channel chan,
                             struct sensor_value *val) {
  return 0;
}


static int sps30_attr_set(const struct device *dev, enum sensor_channel chan,
                          enum sensor_attribute attr,
                          const struct sensor_value *val) {
  return 0;
}


static int16_t sps30_read_product_type(const struct device *dev,
                                       int8_t *product_type,
                                       uint16_t product_type_size) {
  uint16_t local_offset = 0;
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset = sps30_add_command16_to_buffer(buffer_ptr, local_offset,
                                               SPS30_READ_PRODUCT_TYPE_CMD_ID);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    LOG_ERR("Failed to write ready status data CMD (%i)", local_error);
    return local_error;
  }
  local_error = sps30_i2c_read_data_inplace(dev, buffer_ptr, 8);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  sps30_copy_bytes(&buffer_ptr[0], (uint8_t *)product_type, product_type_size);
  return local_error;
}

static int16_t sps30_read_serial_number(const struct device *dev,
                                        int8_t *serial_number,
                                        uint16_t serial_number_size) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset = sps30_add_command16_to_buffer(buffer_ptr, local_offset,
                                               SPS30_READ_SERIAL_NUMBER_CMD_ID);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  local_error = sps30_i2c_read_data_inplace(dev, buffer_ptr, 32);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  sps30_copy_bytes(&buffer_ptr[0], (uint8_t *)serial_number,
                   serial_number_size);
  return local_error;
}

static int16_t sps30_clear_device_status_register(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0xd210);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(5);
  return local_error;
}

static int16_t sps30_device_reset(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0xd304);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(100);
  return local_error;
}

static int16_t sps30_read_firmware_version(const struct device *dev,
                                           uint8_t *major_version,
                                           uint8_t *minor_version) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0xd100);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  local_error = sps30_i2c_read_data_inplace(dev, buffer_ptr, 2);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *major_version = (uint8_t)buffer_ptr[0];
  *minor_version = (uint8_t)buffer_ptr[1];
  return local_error;
}

static int16_t sps30_read_device_status_register(const struct device *dev,
                                                 uint32_t *device_status) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0xd206);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }

  local_error = sps30_i2c_read_data_inplace(dev, buffer_ptr, 4);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *device_status = sensirion_common_bytes_to_uint32_t(&buffer_ptr[0]);
  return local_error;
}

static int16_t sps30_sleep(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x1001);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(5);
  return local_error;
}

static int16_t sps30_start_fan_cleaning(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x5607);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(5);
  return local_error;
}

static int16_t
sps30_write_auto_cleaning_interval(const struct device *dev,
                                   uint32_t auto_cleaning_interval) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x8004);
  local_offset = sensirion_i2c_add_uint32_t_to_buffer(buffer_ptr, local_offset,
                                                      auto_cleaning_interval);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(20);
  return local_error;
}

static int16_t
sps30_read_auto_cleaning_interval(const struct device *dev,
                                  uint32_t *auto_cleaning_interval) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x8004);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(5);
  local_error = sps30_i2c_read_data_inplace(dev, buffer_ptr, 4);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *auto_cleaning_interval = sensirion_common_bytes_to_uint32_t(&buffer_ptr[0]);
  return local_error;
}

static int16_t sps30_wake_up(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset =
      sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x1103);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(5);
  return local_error;
}

static int16_t sps30_wake_up_sequence(const struct device *dev) {
  int16_t local_error = 0;
  local_error = sps30_wake_up(dev);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  local_error = sps30_wake_up(dev);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  return local_error;
}

static int16_t sps30_start_measurement(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset = sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x10);
  local_offset = sensirion_i2c_add_uint16_t_to_buffer(
      buffer_ptr, local_offset, cfg->measurement_output_format);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(20);
  return local_error;
}

static int16_t sps30_stop_measurement(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset = sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x104);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(20);
  return local_error;
}

static int16_t sps30_read_data_ready_flag(const struct device *dev,
                                          uint16_t *data_ready_flag) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset = sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x202);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  local_error = sps30_i2c_read_data_inplace(dev, buffer_ptr, 2);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *data_ready_flag = sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
  return local_error;
}

static int16_t sps30_read_measurement_values_uint16(
    const struct device *dev, uint16_t *mc_1p0, uint16_t *mc_2p5,
    uint16_t *mc_4p0, uint16_t *mc_10p0, uint16_t *nc_0p5, uint16_t *nc_1p0,
    uint16_t *nc_2p5, uint16_t *nc_4p0, uint16_t *nc_10p0,
    uint16_t *typical_particle_size) {
  int16_t local_error = NO_ERROR;
  uint8_t *buffer_ptr = NULL;
  uint16_t local_offset = 0;
  const struct sps30_config *cfg = dev->config;
  struct sps30_data *data = dev->data;
  buffer_ptr = &data->communication_buffer[0];
  local_offset = sps30_add_command16_to_buffer(buffer_ptr, local_offset, 0x300);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  local_error = sps30_i2c_read_data_inplace(dev, buffer_ptr, 20);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *mc_1p0 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
  *mc_2p5 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[2]);
  *mc_4p0 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[4]);
  *mc_10p0 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[6]);
  *nc_0p5 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[8]);
  *nc_1p0 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[10]);
  *nc_2p5 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[12]);
  *nc_4p0 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[14]);
  *nc_10p0 = sensirion_common_bytes_to_uint16_t(&buffer_ptr[16]);
  *typical_particle_size = sensirion_common_bytes_to_uint16_t(&buffer_ptr[18]);
  return local_error;
}

#define SPS30_CONFIG_I2C(inst)                                                 \
  {                                                                            \
      .spec = I2C_DT_SPEC_INST_GET(inst),                                      \
      .bus_io = &sps30_bus_io_i2c,                                             \
      .measurement_output_format =                                             \
          DT_INST_PROP(inst, measurement_output_format),                       \
  }
#define SPS30_DEFINE(inst)                                                     \
  static struct sps30_data sps30_data_##inst;                                  \
  static const struct sps30_config sps30_config_##inst =                       \
      SPS30_CONFIG_I2C(inst);                                                  \
  SENSOR_DEVICE_DT_INST_DEFINE(                                                \
      inst, sps30_chip_init, NULL, &sps30_data_##inst, &sps30_config_##inst,   \
      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &sensor_sps30_api);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(SPS30_DEFINE)
