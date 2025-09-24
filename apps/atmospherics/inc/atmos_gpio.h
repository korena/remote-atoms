#ifndef _ATMOS_GPIO_H_
#define _ATMOS_GPIO_H_
#include <errno.h>
#include <zephyr/drivers/gpio.h>

enum ATMOS_GPIO_OUTPUT {
  LED_GREEN,
  LED_BLUE
};

error_t gpio_init(void);
error_t gpio_toggle_output(enum ATMOS_GPIO_OUTPUT output);
#endif // !_ATMOS_GPIO_H_
//
