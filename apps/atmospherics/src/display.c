#include "display.h"
#include "bme_sensor.h"
#include "fonts/cfb_font_1016.h"
#include "scd41_sensor.h"
#include "zephyr/sys/printk.h"
#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

const struct device *sh1107_dev = NULL;
curr_display_page_t current_display_page = PAGE_C;

error_t init_display_device(void) {
  sh1107_dev = DEVICE_DT_GET_ANY(sinowealth_sh1107);

  if (sh1107_dev == NULL) {
    printk("Error: no SH1107 Display device found.\n");
    return -ENODEV;
  }

  if (!device_is_ready(sh1107_dev)) {
    printk("\nError: Device \"%s\" is not ready; "
           "check the driver initialization logs for errors.\n",
           sh1107_dev->name);
    return EIO;
  }
  printk("Found device \"%s\"\n", sh1107_dev->name);

  return 0;
}
error_t init_framebuffer(void) {
  if (display_set_pixel_format(sh1107_dev, PIXEL_FORMAT_MONO10) != 0) {
    if (display_set_pixel_format(sh1107_dev, PIXEL_FORMAT_MONO01) != 0) {
      printf("Failed to set required pixel format");
      return 0;
    }
  }
  if (cfb_framebuffer_init(sh1107_dev)) {
    printf("Framebuffer initialization failed!\n");
    return 0;
  }
  cfb_framebuffer_clear(sh1107_dev, true);
  display_blanking_off(sh1107_dev);
  int err = cfb_print(sh1107_dev, "Init", 0, 0);
  if (err) {
    printk("Could print (err %d)\n", err);
    return 0;
  }
  err = cfb_print(sh1107_dev, "Complete", 0, 8);
  if (err) {
    printk("Could print (err %d)\n", err);
    return 0;
  }
  cfb_framebuffer_invert(sh1107_dev);
  // cfb_set_kerning(sh1107_dev, 2);
  cfb_framebuffer_finalize(sh1107_dev);
  cfb_framebuffer_clear(sh1107_dev, true);
  return 0;
}

error_t printd(char *str, uint8_t row) {
  int err = cfb_print(sh1107_dev, str, 0, row * 16);
  if (err) {
    printk("Could print (err %d)\n", err);
    return -1;
  }
  return cfb_framebuffer_finalize(sh1107_dev);
}

error_t display_clear(void) {
  cfb_framebuffer_clear(sh1107_dev, true);
  return 0;
}

error_t set_display_choice(curr_display_page_t page) {
  current_display_page = page;
  return 0;
}
error_t get_display_choice(curr_display_page_t *page) {
  *page = current_display_page;
  return 0;
}
error_t display_render_page(void) {
  switch (current_display_page) {
  case PAGE_A:
    return display_page_a();
  case PAGE_B:
    return display_page_b();
  case PAGE_C:
    return display_page_c();
  default:
    return -1;
  }
}

error_t display_page_a(void) { 
  display_clear();
  printd("Page A", 0);
  return 0;
}
error_t display_page_b(void) {
  display_clear();
  printd("Page B", 0);
  return 0;
}
error_t display_page_c(void) {
  struct bme280_data bme280_data;
  struct scd41_data scd41_data;
  char bme_str[24];
  int ret;
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
  printd("Temp: ", 0);
  snprintf(bme_str, sizeof(bme_str), "%d.%03d", bme280_data.temp.val1,
           bme280_data.temp.val2);
  printd(bme_str, 1);
  memset(bme_str, '\0', sizeof(bme_str));
  printd("Press:", 2);
  snprintf(bme_str, sizeof(bme_str), "%d.%03d", bme280_data.press.val1,
           bme280_data.press.val2);
  printd(bme_str, 3);
  memset(bme_str, '\0', sizeof(bme_str));
  printd("Hum:  ", 4);
  snprintf(bme_str, sizeof(bme_str), "%d.%03d", bme280_data.humidity.val1,
           bme280_data.humidity.val2);
  printd(bme_str, 5);
  memset(bme_str, '\0', sizeof(bme_str));
  printd("CO2:  ", 6);
  snprintf(bme_str, sizeof(bme_str), "%d.%03d",
           scd41_data.co2_concentration.val1,
           scd41_data.co2_concentration.val2);
  printd(bme_str, 7);
  memset(bme_str, '\0', sizeof(bme_str));
  return 0;
}
