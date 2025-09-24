#include "bme_sensor.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/sys/printk.h"
#include <errno.h>
#include <zephyr/device.h>

const struct device *bme280_dev = NULL;

error_t init_bme280_device(void) {
  bme280_dev = DEVICE_DT_GET_ANY(bosch_bme280);

  if (bme280_dev == NULL) {
    /* No such node, or the node does not have status "okay". */
    printk("\nError: no BME280 device found.\n");
    return ENODEV;
  }

  if (!device_is_ready(bme280_dev)) {
    printk("\nError: Device \"%s\" is not ready; "
           "check the driver initialization logs for errors.\n",
           bme280_dev->name);
    return EIO;
  }

  printk("Found device \"%s\"\n", bme280_dev->name);
  return 0;
}

error_t get_bme280_data(struct bme280_data *data) {
  if (data == NULL) {
    printf("%s: data parameter is NULL\n", __func__);
    return EINVAL;
  }
  sensor_sample_fetch(bme280_dev);
  sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &data->temp);
  sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &data->press);
  sensor_channel_get(bme280_dev, SENSOR_CHAN_HUMIDITY, &data->humidity);
  printk("<BME280>\nTemperature [°C]: %d.%06d\nPressure [hPa]: %d.%06d "
         "\nHumidity [%%RH] : %d.%06d\n",
         data->temp.val1, data->temp.val2, data->press.val1, data->press.val2,
         data->humidity.val1, data->humidity.val2);
  return 0;
}
