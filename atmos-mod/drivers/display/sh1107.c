#include "sh1107.h"
#include "zephyr/logging/log.h"
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(SH1107, CONFIG_DISPLAY_LOG_LEVEL);

#define DT_DRV_COMPAT sinowealth_sh1107

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "SH1107 display driver enabled without any devices"
#endif

struct sh1107_data {
  uint8_t contrast;
  uint8_t scan_mode;
};

typedef int (*sh1107_bus_check_fn)(const struct device *dev);
typedef int (*sh1107_reg_read_fn)(const struct device *dev, uint8_t start,
                                  uint8_t *buf, int size);
typedef int (*sh1107_reg_write_fn)(const struct device *dev, uint8_t reg,
                                   uint8_t val);

struct sh1107_bus_io {
  sh1107_bus_check_fn check;
  sh1107_reg_read_fn read;
  sh1107_reg_write_fn write;
};

struct sh1107_config {
  struct i2c_dt_spec spec;
  struct sh1107_bus_io *bus_io;
  uint16_t height;
  uint16_t width;
  uint8_t page_offset;
  uint8_t display_offset;
  uint8_t multiplex_ratio;
  uint8_t prechargep;
  bool segment_remap;
  bool com_invdir;
  bool com_sequential;
  bool color_inversion;
  bool sh1106_compatible;
  int ready_time_ms;
};

static struct display_driver_api sh1107_driver_api = {
    //	.blanking_on = sh1107_suspend,
    //	.blanking_off = sh1107_resume,
    //	.write = sh1107_write,
    //	.read = sh1107_read,
    //	.get_framebuffer = sh1107_get_framebuffer,
    //	.set_brightness = sh1107_set_brightness,
    //	.set_contrast = sh1107_set_contrast,
    //	.get_capabilities = sh1107_get_capabilities,
    //	.set_pixel_format = sh1107_set_pixel_format,
    //	.set_orientation = sh1107_set_orientation,
};

//=========================================== Private APIs
static int sh1107_init(const struct device *dev);
static inline bool sh1107_bus_ready(const struct device *dev);
static int sh1107_init_device(const struct device *dev);
static inline int sh1107_bus_check(const struct device *dev);
static inline int sh1107_reg_read(const struct device *dev, uint8_t start,
                                  uint8_t *buf, int size);
static inline int sh1107_reg_write(const struct device *dev, uint8_t reg,
                                   uint8_t val);
// ========================================== Implementation

struct sh1107_bus_io sh1107_bus_io_i2c = {
    .check = sh1107_bus_check,
    .read = sh1107_reg_read,
    .write = sh1107_reg_write,
};

static inline int sh1107_bus_check(const struct device *dev) {
  const struct sh1107_config *cfg = dev->config;
  return device_is_ready(cfg->spec.bus) ? 0 : -ENODEV;
}

static inline int sh1107_reg_read(const struct device *dev, uint8_t start,
                                  uint8_t *buf, int size) {
  const struct sh1107_config *cfg = dev->config;
  return i2c_burst_read_dt(&cfg->spec, start, buf, size);
}

static inline int sh1107_reg_write(const struct device *dev, uint8_t reg,
                                   uint8_t val) {
  const struct sh1107_config *cfg = dev->config;
  return i2c_reg_write_byte_dt(&cfg->spec, reg, val);
}

static int sh1107_init_device(const struct device *dev) {
  //	const struct sh1107_config *config = dev->config;
  //
  //	uint8_t cmd_buf[] = {
  //		SSD1306_SET_ENTIRE_DISPLAY_OFF,
  //		(config->color_inversion ? SSD1306_SET_REVERSE_DISPLAY
  //					 : SSD1306_SET_NORMAL_DISPLAY),
  //	};
  //
  //	/* Reset if pin connected */
  //	if (config->reset.port) {
  //		k_sleep(K_MSEC(SSD1306_RESET_DELAY));
  //		gpio_pin_set_dt(&config->reset, 1);
  //		k_sleep(K_MSEC(SSD1306_RESET_DELAY));
  //		gpio_pin_set_dt(&config->reset, 0);
  //	}
  //
  //	/* Turn display off */
  //	if (ssd1306_suspend(dev)) {
  //		return -EIO;
  //	}
  //
  //	if (ssd1306_set_timing_setting(dev)) {
  //		return -EIO;
  //	}
  //
  //	if (ssd1306_set_hardware_config(dev)) {
  //		return -EIO;
  //	}
  //
  //	if (ssd1306_set_panel_orientation(dev)) {
  //		return -EIO;
  //	}
  //
  //	if (ssd1306_set_charge_pump(dev)) {
  //		return -EIO;
  //	}
  //
  //	if (ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true)) {
  //		return -EIO;
  //	}
  //
  //	if (ssd1306_set_contrast(dev, CONFIG_SSD1306_DEFAULT_CONTRAST)) {
  //		return -EIO;
  //	}
  //
  //	ssd1306_resume(dev);
  return 0;
}

static int sh1107_init(const struct device *dev) {
  const struct sh1107_config *config = dev->config;

  LOG_DBG("");

  k_sleep(K_TIMEOUT_ABS_MS(config->ready_time_ms));

  if (!sh1107_bus_ready(dev)) {
    LOG_ERR("Bus device %s not ready!", dev->name);
    return -EINVAL;
  }

  //	if (config->reset.port) {
  //		int ret;
  //
  //		ret = gpio_pin_configure_dt(&config->reset,
  //					    GPIO_OUTPUT_INACTIVE);
  //		if (ret < 0) {
  //			return ret;
  //		}
  //	}

  if (sh1107_init_device(dev)) {
    LOG_ERR("Failed to initialize device!");
    return -EIO;
  }

  return 0;
}

static inline bool sh1107_bus_ready(const struct device *dev) {
  const struct sh1107_config *config = dev->config;

  return config->bus_io->check(dev);
}

#define SH1107_CONFIG_I2C(inst)                                                \
  {.spec = I2C_DT_SPEC_INST_GET(inst),                                         \
   .bus_io = &sh1107_bus_io_i2c,                                               \
   .height = DT_INST_PROP(inst, height),                                       \
   .width = DT_INST_PROP(inst, width),                                         \
   .page_offset = DT_INST_PROP(inst, page_offset),                             \
   .display_offset = DT_INST_PROP(inst, display_offset),                       \
   .multiplex_ratio = DT_INST_PROP(inst, multiplex_ratio),                     \
   .prechargep = DT_INST_PROP(inst, prechargep),                               \
   .segment_remap = DT_INST_PROP(inst, segment_remap),                         \
   .com_invdir = DT_INST_PROP(inst, com_invdir),                               \
   .com_sequential = DT_INST_PROP(inst, com_sequential),                       \
   .color_inversion = DT_INST_PROP(inst, inversion_on),                        \
   .ready_time_ms = DT_INST_PROP(inst, ready_time_ms)}

// clang-format off
#define SH1107_DEFINE(inst)                                                          \
  static const struct sh1107_config sh1107_config##inst = SH1107_CONFIG_I2C(inst);   \
  static struct sh1107_data sh1107_data##inst;                                       \
  DEVICE_DT_INST_DEFINE(inst,&sh1107_init, NULL,                                     \
                        &sh1107_data##inst,                                          \
                        &sh1107_config##inst,                                        \
                        POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,                   \
                        &sh1107_driver_api);
// clang-format on
DT_INST_FOREACH_STATUS_OKAY(SH1107_DEFINE)
