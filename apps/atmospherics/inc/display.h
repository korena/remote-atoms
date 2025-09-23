#ifndef _ATMOS_DISPLAY_H_
#define _ATMOS_DISPLAY_H_

#include <errno.h>
#include <stdint.h>
#include <zephyr/types.h>


error_t init_display_device(void);
error_t display_print(uint16_t x, uint16_t y);

#endif // !_ATMOS_DISPLAY_H_
