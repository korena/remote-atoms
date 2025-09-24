#include "display.h"
#include "fonts/cfb_font_1016.h"
#include "zephyr/sys/printk.h"
#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

const struct device *sh1107_dev = NULL;

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

error_t page_c(const struct sensor_value *temp,
               const struct sensor_value *humidity,
               const struct sensor_value *press,
               const struct sensor_value *co2) {
  char bme_str[24];
  printd("Temp:", 0);
  snprintf(bme_str, sizeof(bme_str), "%d.%03d", temp->val1, temp->val2);
  printd(bme_str, 1);
  memset(bme_str, '\0', sizeof(bme_str));
  printd("Press:", 2);
  snprintf(bme_str, sizeof(bme_str), "%d.%03d", press->val1, press->val2);
  printd(bme_str, 3);
  memset(bme_str, '\0', sizeof(bme_str));
  printd("Hum:", 4);
  snprintf(bme_str, sizeof(bme_str), "%d.%03d", humidity->val1, humidity->val2);
  printd(bme_str, 5);
  memset(bme_str, '\0', sizeof(bme_str));
  printd("CO2:", 6);
  snprintf(bme_str, sizeof(bme_str), "%d.%03d", co2->val1, co2->val2);
  printd(bme_str, 7);
  memset(bme_str, '\0', sizeof(bme_str));
  return 0;
}
