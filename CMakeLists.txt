cmake_minimum_required(VERSION 3.16)
project(wiringPiLite)

include_directories("./inc/")

set(CMAKE_C_COMPILER clang)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_GNU_SOURCE -pipe -fPIC -pedantic -Wall -Werror -O2")

set(SRC_LIST ${SRC_LIST} src/softPwm.c)
set(SRC_LIST ${SRC_LIST} src/softTone.c)
set(SRC_LIST ${SRC_LIST} src/wiringPi.c)
set(SRC_LIST ${SRC_LIST} src/wiringPiI2C.c)
set(SRC_LIST ${SRC_LIST} src/mcp23017.c)
set(SRC_LIST ${SRC_LIST} src/pcf8574.c)
set(SRC_LIST ${SRC_LIST} src/ads1115.c)
set(SRC_LIST ${SRC_LIST} src/lcd.c)
set(SRC_LIST ${SRC_LIST} src/wiringSerial.c)

add_library(${PROJECT_NAME} STATIC ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} -lm -lpthread -lrt -lcrypt)