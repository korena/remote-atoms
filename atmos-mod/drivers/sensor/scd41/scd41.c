#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/types.h>

#include "scd41.h"
#include "zephyr/sys/printk.h"
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
#define CRC8_LEN (0)

LOG_MODULE_REGISTER(SCD41, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT sensirion_scd41

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "SCD41 driver enabled without any devices"
#endif

struct scd41_data {
  uint16_t serial_number;
  scd4x_sensor_variant variant;
  uint16_t co2_concentration;
  uint32_t temperature;
  uint16_t relative_humidity;
};

struct scd41_config {
  struct i2c_dt_spec spec;
  const struct scd41_bus_io *bus_io;
};

static int scd41_sample_fetch(const struct device *dev,
                              enum sensor_channel chan);
static int scd41_channel_get(const struct device *dev, enum sensor_channel chan,
                             struct sensor_value *val);
static const struct sensor_driver_api sensor_scd41_api = {
    .sample_fetch = scd41_sample_fetch,
    .channel_get = scd41_channel_get,
};

// Sensor basic API
// ---------------------------------------------------------------------------------------------------
static inline int scd41_bus_check(const struct device *dev);
static inline int scd41_reg_read(const struct device *dev, uint8_t start,
                                 uint8_t *buf, int size);
static inline int scd41_reg_write(const struct device *dev, uint8_t reg,
                                  uint8_t val);
// Private functions
// --------------------------------------------------------------------------------------------
int scd41_chip_init(const struct device *dev);

static int16_t scd41_get_data_ready_status(const struct device *dev,
                                           bool *arg_0);
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
static int16_t scd4x_get_sensor_variant_raw(const struct device *dev,
                                            uint16_t *sensor_variant);
static int16_t scd4x_get_sensor_variant(const struct device *dev,
                                        scd4x_sensor_variant *a_sensor_variant);

static int16_t scd41_start_periodic_measurement(const struct device *dev);
static int16_t scd41_stop_periodic_measurement(const struct device *dev);
static int16_t scd41_read_measurement_raw(const struct device *dev,
                                          uint16_t *co2_concentration,
                                          uint16_t *temperature,
                                          uint16_t *relative_humidity);
static int16_t scd41_read_measurement(const struct device *dev, uint16_t *co2,
                                      int32_t *temperature_m_deg_c,
                                      int32_t *humidity_m_percent_rh);
static int16_t scd41_wake_up(const struct device *dev);

static int16_t scd41_reinit(const struct device *dev);

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void scd41_sleep_usec(uint32_t useconds) {
  int32_t remaining = useconds;
  while (remaining > 0) {
    remaining = k_usleep(remaining);
  }
}

const struct scd41_bus_io scd41_bus_io_i2c = {
    .check = scd41_bus_check,
    .read = scd41_reg_read,
    .write = scd41_reg_write,
};

int scd41_chip_init(const struct device *dev) {
  struct scd41_data *data = dev->data;
  int err;

  err = scd41_bus_check(dev);
  if (err < 0) {
    LOG_DBG("bus check failed: %d", err);
    return err;
  }

  err = scd4x_get_sensor_variant(dev, &data->variant);

  if (err < 0) {
    LOG_DBG("Could not get SCD4x chip variant (%d)", err);
    return err;
  }

  if (data->variant != SCD4X_SENSOR_VARIANT_SCD41) {
    LOG_DBG("SCD4x variant not supported (0x%x)", data->variant);
    return -ENOTSUP;
  }
  LOG_DBG("SCD4x variant(0x%x)", data->variant);

  err = scd41_wake_up(dev);

  if (err < 0) {
    LOG_DBG("Could not wake up SCD4x chip (%d)", err);
    return err;
  }

  err = scd41_stop_periodic_measurement(dev);

  if (err < 0) {
    LOG_DBG("Could not stop periodic measurement in SCD4x chip (%d)", err);
    return err;
  }

  err = scd41_reinit(dev);

  if (err < 0) {
    LOG_DBG("Could not reinit SCD4x chip (%d)", err);
    return err;
  }

  err = scd41_get_serial_number(dev, &data->serial_number, 3);
  if (err < 0) {
    LOG_DBG("ID read failed: %d", err);
    return err;
  }

  LOG_DBG("SCD41(0x%x)", data->serial_number);

  //  err = scd41_start_periodic_measurement(dev);
  //  if (err < 0) {
  //    LOG_DBG("Failed to start periodic measurement: %d", err);
  //    return err;
  //  }

  LOG_DBG("\"%s\" OK", dev->name);
  return 0;
}

static int scd41_sample_fetch(const struct device *dev,
                              enum sensor_channel chan) {
  struct scd41_data *data = dev->data;
  int16_t error = NO_ERROR;
  bool data_ready = false;
  uint16_t co2_concentration = 0;
  int32_t temperature = 0;
  int32_t relative_humidity = 0;
  uint16_t repetition = 0;
  int ret;

  __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

  ret = scd41_read_measurement(dev, &co2_concentration, &temperature,
                               &relative_humidity);
  if (ret < 0) {
    printk("Failed to read measurement (%d)", ret);
    return ret;
  }

  for (repetition = 0; repetition < 50; repetition++) {
    //
    // Slow down the sampling to 0.2Hz.
    //
    scd41_sleep_usec(5000000);
    //
    // If ambient pressure compensation during measurement
    // is required, you should call the respective functions here.
    // Check out the header file for the function definition.
    error = scd41_get_data_ready_status(dev, &data_ready);
    if (error != NO_ERROR) {
      printk("error executing get_data_ready_status(): %i\n", error);
      continue;
    }
    while (!data_ready) {
      scd41_sleep_usec(100000);
      error = scd41_get_data_ready_status(dev, &data_ready);
      if (error != NO_ERROR) {
        printf("error executing get_data_ready_status(): %i\n", error);
        continue;
      }
    }
    error = scd41_read_measurement(dev, &co2_concentration, &temperature,
                                   &relative_humidity);
    if (error != NO_ERROR) {
      printf("error executing read_measurement(): %i\n", error);
      continue;
    }
    // Print results in physical units.
    printf("CO2 concentration [ppm]: %u\n", co2_concentration);
    printf("Temperature [m°C] : %i\n", temperature);
    printf("Humidity [mRH]: %i\n", relative_humidity);
  }

  data->temperature = temperature;
  data->relative_humidity = relative_humidity;
  data->co2_concentration = co2_concentration;
  return NO_ERROR;
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

  error = i2c_read(cfg->spec.bus, buffer_ptr, size, cfg->spec.addr);
  if (error) {
    return error;
  }
  for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {

    error = scd41_i2c_check_crc(&buffer_ptr[i], SENSIRION_WORD_SIZE,
                                buffer_ptr[i + SENSIRION_WORD_SIZE]);
    if (error) {
      return error;
    }
    buffer_ptr[j++] = buffer_ptr[i];
    buffer_ptr[j++] = buffer_ptr[i + 1];
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
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(1);
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
  local_error = scd41_get_data_ready_status_raw(dev, &data_ready_status);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *arg_0 = (data_ready_status & 2047) != 0;
  return local_error;
}

static int16_t scd41_start_periodic_measurement(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t buffer_ptr[9];
  uint16_t local_offset = 0;
  const struct scd41_config *cfg = dev->config;
  local_offset = scd41_add_command16_to_buffer(
      buffer_ptr, local_offset, SCD4X_START_PERIODIC_MEASUREMENT_CMD_ID);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  return local_error;
}

static int16_t scd41_stop_periodic_measurement(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t buffer_ptr[9];
  uint16_t local_offset = 0;
  const struct scd41_config *cfg = dev->config;
  local_offset = scd41_add_command16_to_buffer(
      buffer_ptr, local_offset, SCD4X_STOP_PERIODIC_MEASUREMENT_CMD_ID);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(500);
  return local_error;
}

uint16_t sensirion_common_bytes_to_uint16_t(const uint8_t *bytes) {
  return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

uint32_t sensirion_common_bytes_to_uint32_t(const uint8_t *bytes) {
  return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
         (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

int16_t sensirion_common_bytes_to_int16_t(const uint8_t *bytes) {
  return (int16_t)sensirion_common_bytes_to_uint16_t(bytes);
}

int32_t sensirion_common_bytes_to_int32_t(const uint8_t *bytes) {
  return (int32_t)sensirion_common_bytes_to_uint32_t(bytes);
}

static int16_t scd41_read_measurement_raw(const struct device *dev,
                                          uint16_t *co2_concentration,
                                          uint16_t *temperature,
                                          uint16_t *relative_humidity) {
  int16_t local_error = NO_ERROR;
  uint8_t buffer_ptr[9];
  uint16_t local_offset = 0;
  const struct scd41_config *cfg = dev->config;
  local_offset = scd41_add_command16_to_buffer(
      buffer_ptr, local_offset, SCD4X_READ_MEASUREMENT_RAW_CMD_ID);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  scd41_sleep_usec(1 * 1000);
  local_error = scd41_i2c_read_data_inplace(dev, buffer_ptr, 6);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *co2_concentration = sensirion_common_bytes_to_uint16_t(&buffer_ptr[0]);
  *temperature = sensirion_common_bytes_to_uint16_t(&buffer_ptr[2]);
  *relative_humidity = sensirion_common_bytes_to_uint16_t(&buffer_ptr[4]);
  return local_error;
}

static int16_t scd41_read_measurement(const struct device *dev, uint16_t *co2,
                                      int32_t *temperature_m_deg_c,
                                      int32_t *humidity_m_percent_rh) {
  int16_t error;
  uint16_t temperature;
  uint16_t humidity;
  error = scd41_read_measurement_raw(dev, co2, &temperature, &humidity);
  if (error) {
    return error;
  }
  *temperature_m_deg_c = ((21875 * (int32_t)temperature) >> 13) - 45000;
  *humidity_m_percent_rh = ((12500 * (int32_t)humidity) >> 13);
  return NO_ERROR;
}

static int16_t scd41_wake_up(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t buffer_ptr[9];
  uint16_t local_offset = 0;
  const struct scd41_config *cfg = dev->config;
  local_offset = scd41_add_command16_to_buffer(buffer_ptr, local_offset,
                                               SCD4X_WAKE_UP_CMD_ID);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(30);
  return local_error;
}

static int16_t scd41_reinit(const struct device *dev) {
  int16_t local_error = NO_ERROR;
  uint8_t buffer_ptr[9];
  uint16_t local_offset = 0;
  const struct scd41_config *cfg = dev->config;
  local_offset = scd41_add_command16_to_buffer(buffer_ptr, local_offset,
                                               SCD4X_REINIT_CMD_ID);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(30);
  return local_error;
}

static inline int scd41_bus_check(const struct device *dev) {
  const struct scd41_config *cfg = dev->config;
  return device_is_ready(cfg->spec.bus) ? 0 : -ENODEV;
}

static inline int scd41_reg_read(const struct device *dev, uint8_t start,
                                 uint8_t *buf, int size) {
  const struct scd41_config *cfg = dev->config;
  return i2c_burst_read_dt(&cfg->spec, start, buf, size);
}

static inline int scd41_reg_write(const struct device *dev, uint8_t reg,
                                  uint8_t val) {
  const struct scd41_config *cfg = dev->config;
  return i2c_reg_write_byte_dt(&cfg->spec, reg, val);
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
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(1);
  local_error = scd41_i2c_read_data_inplace(dev, buffer_ptr, 6);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  scd41_copy_bytes(&buffer_ptr[0], (uint8_t *)serial_number,
                   (serial_number_size * 2));
  return local_error;
}

static int16_t
scd4x_get_sensor_variant(const struct device *dev,
                         scd4x_sensor_variant *a_sensor_variant) {
  scd4x_sensor_variant ret_val = SCD4X_SENSOR_VARIANT_MASK;
  uint16_t raw_sensor_variant = 0;
  uint16_t my_sensor_variant = 0;
  int16_t local_error = 0;
  ret_val = SCD4X_SENSOR_VARIANT_MASK;
  uint16_t mask = (uint16_t)(ret_val);
  local_error = scd4x_get_sensor_variant_raw(dev, &raw_sensor_variant);
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

static int16_t scd4x_get_sensor_variant_raw(const struct device *dev,
                                            uint16_t *sensor_variant) {
  int16_t local_error = NO_ERROR;
  uint8_t buffer_ptr[9] = {0};
  uint16_t local_offset = 0;
  const struct scd41_config *cfg = dev->config;
  local_offset = scd41_add_command16_to_buffer(
      buffer_ptr, local_offset, SCD4X_GET_SENSOR_VARIANT_RAW_CMD_ID);
  local_error =
      i2c_write(cfg->spec.bus, buffer_ptr, local_offset, cfg->spec.addr);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  k_msleep(1);
  local_error = scd41_i2c_read_data_inplace(dev, buffer_ptr, 2);
  if (local_error != NO_ERROR) {
    return local_error;
  }
  *sensor_variant = scd41_bytes_to_uint16_t(&buffer_ptr[0]);
  return local_error;
}

#define SCD41_CONFIG_I2C(inst)                                                 \
  { .spec = I2C_DT_SPEC_INST_GET(inst), .bus_io = &scd41_bus_io_i2c, }

/*
 * Main instantiation macro.
 */
#define SCD41_DEFINE(inst)                                                     \
  static struct scd41_data scd41_data_##inst;                                  \
  static const struct scd41_config scd41_config_##inst =                       \
      SCD41_CONFIG_I2C(inst);                                                  \
  SENSOR_DEVICE_DT_INST_DEFINE(                                                \
      inst, scd41_chip_init, NULL, &scd41_data_##inst, &scd41_config_##inst,   \
      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &sensor_scd41_api);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(SCD41_DEFINE)
