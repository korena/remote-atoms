#include "atmos_gpio.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <zephyr/devicetree.h>

#define SW0_NODE DT_ALIAS(sw0)
#define DEBOUNCE_DELAY_MS 50

const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback sw0_cb_data;

// Struct for holding the workqueue
static struct k_work_delayable button_work;

// Work handler: button pressed
static void button_work_handler(struct k_work *work) {
  int state;
  // Read the state of the button (after the debounce delay)
  state = gpio_pin_get_dt(&sw0);
  if (state < 0) {
    printk("Error (%d): failed to read button pin\r\n", state);
  } else if (state) {
    printk("Doing some work...now with debounce!\r\n");
  }
}

static void gpio_isr(const struct device *dev, struct gpio_callback *cb,
              unsigned int pins) {
  k_work_reschedule(&button_work, K_MSEC(DEBOUNCE_DELAY_MS));
}

error_t gpio_init(void) {
  int ret;
  // Initialize work item
  k_work_init_delayable(&button_work, button_work_handler);
  // Make sure that the button was initialized
  if (!gpio_is_ready_dt(&sw0)) {
    printk("ERROR: button not ready\r\n");
    return 0;
  }

  ret = gpio_pin_configure_dt(&sw0, GPIO_INPUT);
  if (ret < 0) {
    printk("ERROR: Failed to configure button\r\n");
    return 0;
  }

  // Configure the interrupt
  ret = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    printk("ERROR: could not configure button as interrupt source\r\n");
    return 0;
  }
  // Connect callback function (ISR) to interrupt source
  gpio_init_callback(&sw0_cb_data, gpio_isr, BIT(sw0.pin));
  gpio_add_callback(sw0.port, &sw0_cb_data);

  return 0;
}

