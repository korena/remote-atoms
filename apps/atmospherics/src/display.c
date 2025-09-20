#include "display.h"
#include <errno.h>
#include "zephyr/sys/printk.h"
#include <zephyr/device.h>

const struct device *sh1107_dev = NULL;


error_t init_display_device()
{
  sh1107_dev = DEVICE_DT_GET_ANY(sinowealth_sh1107);

  if (sh1107_dev == NULL) {
    printk("Error: no SH1107 Display device found.\n");
    return -ENODEV;
  }

  return 0;
}
