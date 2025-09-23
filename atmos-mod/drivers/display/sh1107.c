#include "sh1107.h"
#include "zephyr/logging/log.h"
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(SH1107, CONFIG_DISPLAY_LOG_LEVEL);

#define DT_DRV_COMPAT sinowealth_sh1107

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "SH1107 display driver enabled without any devices"
#endif

typedef int (*sh1107_bus_check_fn)(const struct device *dev);
typedef int (*sh1107_reg_read_fn)(const struct device *dev, uint8_t start,
                                  uint8_t *buf, int size);
typedef int (*sh1107_write_fn)(const struct device *dev, uint8_t *buf,
                               size_t len, bool command);
struct sh1107_usr_data {
  uint8_t *data;
  size_t len;
};

struct sh1107_data {
  uint8_t contrast;
  uint8_t scan_mode;
};

struct sh1107_bus_io {
  sh1107_bus_check_fn check;
  sh1107_reg_read_fn read;
  sh1107_write_fn write;
};

struct sh1107_config {
  struct i2c_dt_spec spec;
  struct sh1107_bus_io *bus_io;
  struct gpio_dt_spec reset;
  uint16_t height;
  uint16_t width;
  uint8_t segment_offset;
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

//=========================================== Private APIs
static int sh1107_init(const struct device *dev);
static inline bool sh1107_bus_ready(const struct device *dev);
static int sh1107_init_device(const struct device *dev);
// ========================================== bus APIs
static inline int sh1107_bus_check(const struct device *dev);
static inline int sh1107_reg_read(const struct device *dev, uint8_t start,
                                  uint8_t *buf, int size);
static int sh1107_write(const struct device *dev, uint8_t *buf, size_t len,
                        bool command);
static int sh1107_write_data(const struct device *dev, const uint16_t x,
                             const uint16_t y,
                             const struct display_buffer_descriptor *desc,
                             const void *buf);
// ========================================== Display Driver API
static int sh1107_display_suspend(const struct device *dev);
static int sh1107_display_resume(const struct device *dev);
static int sh1107_display_write(const struct device *dev, const uint16_t x,
                                const uint16_t y,
                                const struct display_buffer_descriptor *desc,
                                const void *buf);
static int sh1107_display_read(const struct device *dev, const uint16_t x,
                               const uint16_t y,
                               const struct display_buffer_descriptor *desc,
                               void *buf);
static void *sh1107_display_get_framebuffer(const struct device *dev);

static int sh1107_display_set_brightness(const struct device *dev,
                                         const uint8_t brightness);
static int sh1107_display_set_contrast(const struct device *dev,
                                       const uint8_t contrast);
static void sh1107_display_get_capabilities(const struct device *dev,
                                            struct display_capabilities *caps);

static int sh1107_display_set_pixel_format(const struct device *dev,
                                           const enum display_pixel_format pf);

static int
sh1107_display_set_orientation(const struct device *dev,
                               const enum display_orientation orientation);
// ========================================== Implementation
struct sh1107_bus_io sh1107_bus_io_i2c = {
    .check = sh1107_bus_check,
    .read = sh1107_reg_read,
    .write = sh1107_write,
};

static struct display_driver_api sh1107_driver_api = {
    .blanking_on = sh1107_display_suspend,
    .blanking_off = sh1107_display_resume,
    .write = sh1107_display_write,
    .read = sh1107_display_read,
    .get_framebuffer = sh1107_display_get_framebuffer,
    .set_brightness = sh1107_display_set_brightness,
    .set_contrast = sh1107_display_set_contrast,
    .get_capabilities = sh1107_display_get_capabilities,
    .set_pixel_format = sh1107_display_set_pixel_format,
    .set_orientation = sh1107_display_set_orientation,
};

static inline int sh1107_bus_check(const struct device *dev) {
  const struct sh1107_config *cfg = dev->config;
  return i2c_is_ready_dt(&cfg->spec);
}

static inline int sh1107_reg_read(const struct device *dev, uint8_t start,
                                  uint8_t *buf, int size) {
  const struct sh1107_config *cfg = dev->config;
  return i2c_burst_read_dt(&cfg->spec, start, buf, size);
}

static int sh1107_write(const struct device *dev, uint8_t *buf, size_t len,
                        bool command) {
  const struct sh1107_config *config = dev->config;
  // LOG_HEXDUMP_DBG(buf, len, "raw_buf");
  return i2c_burst_write_dt(&config->spec,
                            command ? SH110X_CONTROL_ALL_BYTES_CMD
                                    : SH110X_CONTROL_ALL_BYTES_DATA,
                            buf, len);
}

static int sh1107_write_data(const struct device *dev, const uint16_t x,
                             const uint16_t y,
                             const struct display_buffer_descriptor *desc,
                             const void *buf) {
  const struct sh1107_config *config = dev->config;
  uint8_t x_offset = x + config->segment_offset;
  uint8_t cmd_buf[] = {
      SH110X_SETLOWCOLUMN | (x_offset & SH110X_SETLOWCOLUMN_MASK),
      SH110X_SETHIGHCOLUMN | ((x_offset >> 4) & SH110X_SETLOWCOLUMN_MASK),
      SH110X_SETPAGEADDR | (y / 8)};
  uint8_t *buf_ptr = (uint8_t *)buf;
  for (uint8_t n = 0; n < desc->height / 8; n++) {
    cmd_buf[sizeof(cmd_buf) - 1] = SH110X_SETPAGEADDR | (n + (y / 8));
    // LOG_HEXDUMP_DBG(cmd_buf, sizeof(cmd_buf), "cmd_buf");
    if (sh1107_write(dev, cmd_buf, sizeof(cmd_buf), true)) {
      return -1;
    }
    if (sh1107_write(dev, buf_ptr, desc->width, false)) {
      return -1;
    }

    buf_ptr = buf_ptr + desc->width;
    if (buf_ptr > ((uint8_t *)buf + desc->buf_size)) {
      LOG_ERR("Exceeded buffer length");
      return -1;
    }
  }
  return 0;
}

static int sh1107_display_suspend(const struct device *dev) {
  uint8_t cmd_buf[] = {
      SH110X_DISPLAYOFF,
  };
  return sh1107_write(dev, cmd_buf, sizeof(cmd_buf), true);
}

static int sh1107_display_resume(const struct device *dev) {
  uint8_t cmd_buf[] = {
      SH110X_DISPLAYON,
  };

  return sh1107_write(dev, cmd_buf, sizeof(cmd_buf), true);
}

static int sh1107_display_write(const struct device *dev, const uint16_t x,
                                const uint16_t y,
                                const struct display_buffer_descriptor *desc,
                                const void *buf) {
  const struct sh1107_config *config = dev->config;
  __ASSERT(desc->width <= desc->pitch, "Pitch is smaller then width");
  __ASSERT(desc->pitch <= config->width,
           "Pitch in descriptor is larger than screen size");
  __ASSERT(desc->height <= config->height,
           "Height in descriptor is larger than screen size");
  __ASSERT(x + desc->pitch <= config->width,
           "Writing outside screen boundaries in horizontal direction");
  __ASSERT(y + desc->height <= config->height,
           "Writing outside screen boundaries in vertical direction");
  // LOG_DBG("x %u, y %u, pitch %u, width %u, height %u, buf_len %u", x, y,
  //        desc->pitch, desc->width, desc->height, desc->buf_size);
  if (desc->width > desc->pitch || x + desc->pitch > config->width ||
      y + desc->height > config->height) {
    LOG_ERR("Pre-conditions failed, buffer desc does not fit LCD");
    LOG_ERR("width (%u) > pitch (%u) || x + pitch (%u) > conf->width (%u) || y + height (%u) > conf->height(%u)",
            desc->width, desc->pitch, x + desc->pitch, config->width, y + desc->height, config->height);
    return -EINVAL;
  }
  return sh1107_write_data(dev, x, y, desc, buf);
}

static int sh1107_display_read(const struct device *dev, const uint16_t x,
                               const uint16_t y,
                               const struct display_buffer_descriptor *desc,
                               void *buf) {
  LOG_ERR("Unsupported");
  return -ENOTSUP;
}

static void *sh1107_display_get_framebuffer(const struct device *dev) {
  LOG_ERR("Unsupported");
  return NULL;
}
static int sh1107_display_set_brightness(const struct device *dev,
                                         const uint8_t brightness) {
  LOG_WRN("Unsupported");
  return -ENOTSUP;
}

static int sh1107_display_set_contrast(const struct device *dev,
                                       const uint8_t contrast) {
  uint8_t cmd_buf[] = {
      SH110X_SETCONTRAST,
      contrast,
  };

  return sh1107_write(dev, cmd_buf, sizeof(cmd_buf), true);
}

static void sh1107_display_get_capabilities(const struct device *dev,
                                            struct display_capabilities *caps) {
  const struct sh1107_config *config = dev->config;
  memset(caps, 0, sizeof(struct display_capabilities));
  caps->x_resolution = config->width;
  caps->y_resolution = config->height;
  caps->supported_pixel_formats = PIXEL_FORMAT_MONO10;
  caps->current_pixel_format = PIXEL_FORMAT_MONO10;
  caps->screen_info = SCREEN_INFO_MONO_VTILED;
  caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int sh1107_display_set_pixel_format(const struct device *dev,
                                           const enum display_pixel_format pf) {
  if (pf == PIXEL_FORMAT_MONO10) {
    return 0;
  }
  LOG_ERR("Unsupported");
  return -ENOTSUP;
}

static int
sh1107_display_set_orientation(const struct device *dev,
                               const enum display_orientation orientation) {
  LOG_ERR("Unsupported");
  return -ENOTSUP;
}

static int sh1107_init_device(const struct device *dev) {
  const struct sh1107_config *config = dev->config;
  // clang-format off
  uint8_t cmd_buf[] = {
      SH110X_DISPLAYOFF,
      SH110X_SETDISPLAYCLOCKDIV,0x51,
      SH110X_MEMORYMODE,
      SH110X_SETCONTRAST,0x4F, // 0x81, 0x4F
      SH110X_DCDC,0x8A,              // 0xAD, 0x8A
      (config->segment_remap?SH110X_SEGREMAP:SH110X_NOOP),
      SH110X_COMSCANINC, // 0xC0
     SH110X_SETDISPSTARTLINE,0x00, // 0xDC 0x00
     SH110X_SETDISPLAYOFFSET,
     config->display_offset,
     SH110X_SETPRECHARGE,
     config->prechargep, // 0xd9, 0x22,
     SH110X_SETVCOMDETECT,0x35, // 0xdb, 0x35,
     SH110X_SETMULTIPLEX,
     config->multiplex_ratio,    // 0xa8, 0x3f,
     SH110X_DISPLAYALLON_RESUME, // 0xa4
     SH110X_SETDISPLAYOFFSET,
     config->display_offset,     // 0xd3, 0x60,
     (config->color_inversion ? SH110X_INVERTDISPLAY : SH110X_NORMALDISPLAY),
  };
  // clang-format on
  /* Reset if pin connected */
  if (config->reset.port) {
    k_sleep(K_MSEC(SH110X_RESET_DELAY));
    gpio_pin_set_dt(&config->reset, 1);
    k_sleep(K_MSEC(SH110X_RESET_DELAY));
    gpio_pin_set_dt(&config->reset, 0);
  }

  k_sleep(K_MSEC(config->ready_time_ms));

  if (config->bus_io->write(dev, cmd_buf, sizeof(cmd_buf), true)) {
    return -EIO;
  }

  k_sleep(K_MSEC(300));
  uint8_t cmd[1] = {SH110X_DISPLAYON};
  if (config->bus_io->write(dev, cmd, sizeof(cmd), true)) {
    return -EIO;
  }
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

  if (config->reset.port) {
    int ret;
    ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
      LOG_ERR("Unable to reset display (%d)", ret);
      return ret;
    }
  }

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

// clang-format off
#define SH1107_CONFIG_I2C(inst)                                                \
  {                                                                            \
    .spec = I2C_DT_SPEC_INST_GET(inst),                                        \
    .reset = GPIO_DT_SPEC_GET_OR(inst, reset_gpios, {0}),                      \
    .bus_io = &sh1107_bus_io_i2c,                                              \
    .height = DT_INST_PROP(inst, height),                                      \
    .width = DT_INST_PROP(inst, width),                                        \
    .segment_offset = DT_INST_PROP(inst, segment_offset),                      \
    .page_offset = DT_INST_PROP(inst, page_offset),                            \
    .display_offset = DT_INST_PROP(inst, display_offset),                      \
    .multiplex_ratio = DT_INST_PROP(inst, multiplex_ratio),                    \
    .prechargep = DT_INST_PROP(inst, prechargep),                              \
    .segment_remap = DT_INST_PROP(inst, segment_remap),                        \
    .com_invdir = DT_INST_PROP(inst, com_invdir),                              \
    .com_sequential = DT_INST_PROP(inst, com_sequential),                      \
    .color_inversion = DT_INST_PROP(inst, inversion_on),                       \
    .ready_time_ms = DT_INST_PROP(inst, ready_time_ms)                         \
  }
// clang-format on

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
