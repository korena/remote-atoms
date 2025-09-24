#include "atmos_app.h"
#include "atmos_gpio.h"
#include "bme_sensor.h"
#include "display.h"
#include "scd41_sensor.h"
#include "atmos_app.h"
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 5000
#define DEBOUNCE_DELAY_MS 50
/*every hour*/
#define SCD41_PRESS_UPDATE_PERIOD_SEC (60 * 60 * 1000)

int run_atmos_app(void) {
  int ret;
  struct bme280_data bme280_data;
  struct scd41_data scd41_data;

  ret = gpio_init();
  if (ret < 0) {
    return 0;
  }

  ret = init_display_device();
  if (ret < 0) {
    return 0;
  }

  ret = init_framebuffer();
  if (ret < 0) {
    return 0;
  }

  ret = init_bme280_device();
  if (ret < 0) {
    return 0;
  }

  ret = init_scd41_device();
  if (ret < 0) {
    return 0;
  }

  // const struct device *sgp40_dev = get_sgp40_device();

  while (1) {
    ret = gpio_toggle_output(LED_GREEN);
    ret = display_render_page();
    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}

