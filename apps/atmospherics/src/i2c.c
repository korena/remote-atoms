#include "i2c.h"

#include "zephyr/sys/printk.h"
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sgp40.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define I2C2_NODE DT_NODELABEL(i2c2)
#define I2C1_NODE DT_NODELABEL(i2c1)

int i2c1_devices_detect(void)
{
  const struct device *i2c_dev = NULL;
  uint8_t data;
  int ret;
  i2c_dev = DEVICE_DT_GET(I2C1_NODE);
  if (i2c_dev == NULL) {
    printk("I2C device not ready\n");
    return 0;
  }

  if (!device_is_ready(i2c_dev)) {
    printk("I2C device not ready\n")  ;
    return 0;
  }
  printk("Scanning I2C1 bus ...\n");

  uint8_t addr[5] = {0x3C};

  for (int i = 0; i < 5; i++) {
  ret = i2c_read(i2c_dev, &data, 1, addr[i]);
    if (ret == 0) {
    printk("Device found at address: 0x%02X\n", addr[i]);
    } else {
      printk("No Device found at address: 0x%02X (%d)\n", addr[i], ret);
    }
  }
  return 0;
}

int i2c2_devices_detect(void)
{
  const struct device *i2c_dev = NULL;
  uint8_t data;
  int ret;
  i2c_dev = DEVICE_DT_GET(I2C2_NODE);
  if (i2c_dev == NULL) {
    printk("I2C device not ready\n");
    return 0;
  }

  if (!device_is_ready(i2c_dev)) {
    printk("I2C device not ready\n")  ;
    return 0;
  }
  printk("Scanning I2C2 bus ...\n");

  uint8_t addr[5] = {0x29, 0x53, 0x59, 0x68, 0x76};

  for (int i = 0; i < 5; i++) {
  ret = i2c_read(i2c_dev, &data, 1, addr[i]);
    if (ret == 0) {
    printk("Device found at address: 0x%02X\n", addr[i]);
    } else {
      printk("No Device found at address: 0x%02X (%d)\n", addr[i], ret);
    }
  }
  return 0;
}
