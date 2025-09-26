cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
include(${ZEPHYR_BASE}/cmake/cfb.cmake NO_POLICY_SCOPE)
project(atmos)

target_sources(app PRIVATE
  src/main.c
  src/atmos_app.c
  src/i2c.c
  src/bme_sensor.c
  src/sgp_sensor.c
  src/scd41_sensor.c
  src/sps30_sensor.c
  src/display.c
  src/atmos_gpio.c
  src/test.c
)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)


