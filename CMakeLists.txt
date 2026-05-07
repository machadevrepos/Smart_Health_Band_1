cmake_minimum_required(VERSION 3.20.0)

set(DTS_ROOT ${DTS_ROOT} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smart_health_band)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_sources(app PRIVATE
  src/main.c
  src/board.c
  src/power.c
  src/ble.c
  src/bmi270.c
  src/tmp117.c
  src/max32664_safe.c
)
