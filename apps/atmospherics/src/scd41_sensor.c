#include "scd41_sensor.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/sys/printk.h"
#include <errno.h>
#include <sys/errno.h>
#include <zephyr/device.h>

const struct device *scd41_dev = NULL;

error_t init_scd41_device(void) {
  scd41_dev = DEVICE_DT_GET(DT_NODELABEL(scd41));
  if (scd41_dev == NULL) {
    printk("\nError: no SCD41 device found.\n");
    return ENODEV;
  }

  if (!device_is_ready(scd41_dev)) {
    printk("\nError: Device \"%s\" is not ready; "
           "check the driver initialization logs for errors.\n",
           scd41_dev->name);
    return EIO;
  }

  printk("Found device \"%s\"\n", scd41_dev->name);
  return 0;
}

error_t update_scd41_pressure(const struct sensor_value *pressure) {
  if (scd41_dev == NULL) {
    printk("\nError: no SCD41 device found.\n");
    return ENODEV;
  }
  return sensor_attr_set(scd41_dev, SENSOR_CHAN_PRESS, SENSOR_ATTR_CALIBRATION,
                         pressure);
}

error_t get_scd41_data(struct scd41_data *scd41_data) {
  if (scd41_dev == NULL) {
    printk("\nError: no SCD41 device found.\n");
    return ENODEV;
  }

  if (0 != sensor_sample_fetch(scd41_dev)) {
    printk("Failed to fetch sample from SCD41 device.\n");
    return 0;
  }
  sensor_channel_get(scd41_dev, SENSOR_CHAN_AMBIENT_TEMP,
                     &scd41_data->temperature);
  sensor_channel_get(scd41_dev, SENSOR_CHAN_HUMIDITY,
                     &scd41_data->relative_humidity);
  sensor_channel_get(scd41_dev, SENSOR_CHAN_CO2,
                     &scd41_data->co2_concentration);

  printk("<SCD41>\nCO2 concentration [ppm]: %d.%07d\nTemperature [°C] : "
         "%d.%06d\nHumidity [%%RH]: %d.%06d\n",
         scd41_data->co2_concentration.val1, scd41_data->co2_concentration.val2,
         scd41_data->temperature.val1, scd41_data->temperature.val2,
         scd41_data->relative_humidity.val1,
         scd41_data->relative_humidity.val2);
  return 0;
}
