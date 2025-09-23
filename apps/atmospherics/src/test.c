#include "test.h"
#include "i2c.h"

int run_test(void) {
  i2c1_devices_detect();
  i2c2_devices_detect();
  return 0;
}
