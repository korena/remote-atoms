#include "atmos_app.h"
#include "bme_sensor.h"
#include "display.h"
#include "scd41_sensor.h"
#include "zephyr/sys/printk.h"
#include "zephyr/sys/util_macro.h"
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
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

//int gpio_init(void) {
//  int ret;
//  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
//  if (ret < 0) {
//    return 0;
//  }
//  ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
//  if (ret < 0) {
//    return 0;
//  }
//
//  // Initialize work item
//  k_work_init_delayable(&button_work, button_work_handler);
//
//  // Make sure that the button was initialized
//  if (!gpio_is_ready_dt(&sw0)) {
//    printk("ERROR: button not ready\r\n");
//    return 0;
//  }
//
//  ret = gpio_pin_configure_dt(&sw0, GPIO_INPUT);
//
//  // enable interrupt on button for rising edge
//  ret = gpio_pin_interrupt_configure(&sw0,BIT(sw0.pin), GPIO_INT_LEVEL_ACTIVE);
//  if (ret != 0) {
//    printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
//           sw0.port->name, sw0.pin);
//    return ret;
//  }
//
//  gpio_init_callback(&sw0_cb_data, gpio_isr, BIT(sw0.pin));
//  gpio_add_callback(sw0.port, &sw0_cb_data);
//
//  return ret;
//}

int run_atmos_app(void) {
  int ret;
  struct bme280_data bme280_data;
  struct scd41_data scd41_data;

  if (!gpio_is_ready_dt(&led) ||
      !gpio_is_ready_dt(&led1)) {
    return 0;
  }

  ret = init_display_device();
  if (ret < 0) {
    return 0;
  }

  ret = display_clear();
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
    ret = gpio_pin_toggle_dt(&led);
    if (ret < 0) {
      return 0;
    }
    ret = gpio_pin_toggle_dt(&led1);
    if (ret < 0) {
      return 0;
    }
    // BME280
    ret = get_bme280_data(&bme280_data);
    if (ret < 0) {
      return 0;
    }
    // SCD41
    ret = get_scd41_data(&scd41_data);
    if (ret < 0) {
      return 0;
    }

    ret = page_c(&bme280_data.temp, &bme280_data.humidity, &bme280_data.press,
                 &scd41_data.co2_concentration);
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
    //			printf("Failed to fetch sample from SGP40
    // device.\n");
    //			return 0;
    //		}
    //
    //		sensor_channel_get(sgp40_dev, SENSOR_CHAN_GAS_RES,
    // &gas);
    //		printf("SHT4X: %.2f Temp. [C] ; %0.2f RH [%%] -- SGP40:
    // %d Gas
    //[a.u.]\n", 		       sensor_value_to_double(&temp),
    //		       sensor_value_to_double(&humidity),
    //		       gas.val1);
    //
    
    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}
