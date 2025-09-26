#include "sps30_sensor.h"
#include "sensor/sps30/sps30.h"
#include "zephyr/drivers/sensor.h"
#include "zephyr/sys/printk.h"
#include <errno.h>
#include <sys/errno.h>
#include <zephyr/device.h>

const struct device *sps30_dev = NULL;

error_t init_sps30_device(void) {
  sps30_dev = DEVICE_DT_GET(DT_NODELABEL(sps30));
  if (sps30_dev == NULL) {
    printk("\nError: no SPS30 device found.\n");
    return ENODEV;
  }
  if (!device_is_ready(sps30_dev)) {
    printk("\nError: Device \"%s\" is not ready; "
           "check the driver initialization logs for errors.\n",
           sps30_dev->name);
    return EIO;
  }
  return 0;
}

error_t get_sps30_data(struct sps30_data *sps30_data) {
  int ret = 0;
  if (sps30_dev == NULL) {
    printk("\nError: no SPS30 device found.\n");
    return ENODEV;
  }
  ret = sensor_sample_fetch(sps30_dev);
  if (0 != ret) {
    printk("Failed to fetch sample from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_MC_1p0,
                           &sps30_data->mc_1p0);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_MC_2p5,
                           &sps30_data->mc_2p5);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_MC_4p0,
                           &sps30_data->mc_4p0);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_MC_10p0,
                           &sps30_data->mc_10p0);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_NC_0p5,
                           &sps30_data->nc_0p5);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_NC_1p0,
                           &sps30_data->nc_1p0);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_NC_2p5,
                           &sps30_data->nc_2p5);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_NC_4p0,
                           &sps30_data->nc_4p0);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_NC_10p0,
                           &sps30_data->nc_10p0);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  ret = sensor_channel_get(sps30_dev, SENSOR_CHAN_PM_TPS,
                           &sps30_data->typical_particle_size);
  if (0 != ret) {
    printk("Failed to get channel value from SPS30 device.\n");
    return ret;
  }
  printk("<SPS30>\nPM_MC 1.0 : %u\n", sps30_data->mc_1p0.val1);
  printk("PM_MC 2.5 : %u\n", sps30_data->mc_2p5.val1);
  printk("PM_MC 4.0 : %u\n", sps30_data->mc_4p0.val1);
  printk("PM_MC 10.0: %u\n", sps30_data->mc_10p0.val1);
  printk("PM_NC 0.5 : %u\n", sps30_data->nc_0p5.val1);
  printk("PM_NC 1.0 : %u\n", sps30_data->nc_1p0.val1);
  printk("PM_NC 2.5 : %u\n", sps30_data->nc_2p5.val1);
  printk("PM_NC 4.0 : %u\n", sps30_data->nc_4p0.val1);
  printk("PM_NC 10.0: %u\n", sps30_data->nc_10p0.val1);
  printk("PM_TPS    : %u\n", sps30_data->typical_particle_size.val1);
  return 0;
}
