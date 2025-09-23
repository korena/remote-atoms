#ifndef _ATMOS_DISPLAY_H_
#define _ATMOS_DISPLAY_H_

#include <errno.h>
#include <stdint.h>
#include <zephyr/types.h>


error_t init_display_device(void);
error_t init_framebuffer(void);
error_t printd(char* str, uint8_t row);
error_t display_clear(void);
#endif // !_ATMOS_DISPLAY_H_
