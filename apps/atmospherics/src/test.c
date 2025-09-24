#include "test.h"
#include "i2c.h"
#include "atmos_gpio.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

int run_test_i2c_detect(void) {
  i2c1_devices_detect();
  i2c2_devices_detect();
  return 0;
}

int run_test_gpio_input(void) {
  int ret = gpio_init();
  if (ret < 0) {
    return 0;
  }
  while (1) {
    k_sleep(K_FOREVER);
  }
  return 0;
}
