#include "display.h"
#include "zephyr/sys/printk.h"
#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>

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

enum corner {
	TOP_LEFT,
	TOP_RIGHT,
	BOTTOM_RIGHT,
	BOTTOM_LEFT
};

static void fill_buffer_mono(enum corner corner, uint8_t grey, uint8_t *buf,
			     size_t buf_size)
{
	uint16_t color;

	switch (corner) {
	case BOTTOM_LEFT:
		color = (grey & 0x01u) ? 0xFFu : 0x00u;
		break;
	default:
		color = 0;
		break;
	}

	memset(buf, color, buf_size);
}

error_t display_print(uint16_t x, uint16_t y) {
  const struct display_driver_api *api;
  struct display_capabilities cap;
  struct display_buffer_descriptor buf_desc;
	uint8_t buf[16] = {0};
	size_t buf_size = 16;

  if (sh1107_dev == NULL) {
    printk("Error: no SH1107 Display device found.\n");
    return -ENODEV;
  }

  api = (const struct display_driver_api *)sh1107_dev->api;
  api->get_capabilities(sh1107_dev, &cap);

	(void)memset(buf, 0xFFu, buf_size);

	buf_desc.buf_size = buf_size;
	buf_desc.pitch = 8;
	buf_desc.width = 8;
	buf_desc.height = 8;
  // Clear screen
	for (int idx = 0; idx < cap.y_resolution; idx += 8) {
		display_write(sh1107_dev, 0, idx, &buf_desc, buf);
    k_msleep(1000);
	}
	for (int idx = 0; idx < cap.x_resolution; idx += 8) {
		display_write(sh1107_dev, idx, 0, &buf_desc, buf);
    k_msleep(1000);
	}
  return 0;
}











