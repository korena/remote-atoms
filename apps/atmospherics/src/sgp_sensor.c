#include "sgp_sensor.h"

#include <zephyr/device.h>

const struct device *get_sgp40_device(void) {
//	const struct device *const sgp = DEVICE_DT_GET_ANY(sensirion_sgp40);
//
//	if (!device_is_ready(sgp)) {
//		printf("Device %s is not ready.\n", sgp->name);
//		return 0;
//	}
//  printk("Found device \"%s\"\n", sgp->name);
//

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
  return NULL;
}
