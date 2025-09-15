#include "scd41_sensor.h"
#include "zephyr/sys/printk.h"
#include <errno.h>
#include <sys/errno.h>
#include <zephyr/device.h>


const struct device *scd41_dev = NULL;

error_t init_scd41_device(void) {
  scd41_dev = DEVICE_DT_GET_ANY(sensirion_scd41);

  if (scd41_dev == NULL) {
    printk("\nError: no SCD41 device found.\n");
    return ENODEV;
  }

  printk("Found device \"%s\"\n", scd41_dev->name);
  return 0;
}
