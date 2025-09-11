/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sgp40.h>
#include <zephyr/kernel.h>

#include "bme_sensor.h"
#include "sgp_sensor.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define I2C2_NODE DT_NODELABEL(i2c2)
#define I2C1_NODE DT_NODELABEL(i2c1)

#if !DT_HAS_COMPAT_STATUS_OKAY(sensirion_sgp40)
#error "No sensirion,sgp40 compatible node found in the device tree"
#endif
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

int main(void) {
  int ret;

  if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&led1)) {
    return 0;
  }
  ret = init_bme280_device();
  if (ret < 0) {
    return 0;
  }

  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return 0;
  }
  ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return 0;
  }

  // const struct device *sgp40_dev = get_sgp40_device();

  while (1) {
    ret = gpio_pin_toggle_dt(&led);
    if (ret < 0) {
      return 0;
    }
    ret = gpio_pin_toggle_dt(&led1);
    if (ret < 0) {
      return 0;
    }
    // BME280
    struct bme280_data bme280_data;
    ret = get_bme280_data(&bme280_data);
    if (ret < 0) {
      return 0;
    }
    printk("temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\n",
           bme280_data.temp.val1, bme280_data.temp.val2, bme280_data.press.val1,
           bme280_data.press.val2, bme280_data.humidity.val1,
           bme280_data.humidity.val2);

    //    // SGP40
    //	  struct sensor_value gas;
    //	  struct sensor_value comp_t;
    //	  struct sensor_value comp_rh;
    //		comp_t.val1 = temp.val1; /* Temp [°C] */
    //		comp_rh.val1 = humidity.val1; /* RH [%] */
    //		sensor_attr_set(sgp40_dev,
    //				SENSOR_CHAN_GAS_RES,
    //				SENSOR_ATTR_SGP40_TEMPERATURE,
    //				&comp_t);
    //		sensor_attr_set(sgp40_dev,
    //				SENSOR_CHAN_GAS_RES,
    //				SENSOR_ATTR_SGP40_HUMIDITY,
    //				&comp_rh);
    //		if (sensor_sample_fetch(sgp40_dev)) {
    //			printf("Failed to fetch sample from SGP40 device.\n");
    //			return 0;
    //		}
    //
    //		sensor_channel_get(sgp40_dev, SENSOR_CHAN_GAS_RES, &gas);
    //		printf("SHT4X: %.2f Temp. [C] ; %0.2f RH [%%] -- SGP40: %d Gas
    //[a.u.]\n", 		       sensor_value_to_double(&temp),
    //		       sensor_value_to_double(&humidity),
    //		       gas.val1);
    //
    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}
