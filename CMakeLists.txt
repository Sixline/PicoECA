cmake_minimum_required(VERSION 3.25)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(picoeca)

pico_sdk_init()

add_executable(picoeca
        picoeca.c
        can2040/can2040.c
        )

#pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/PwmIn.pio)

# pull in common dependencies
target_link_libraries(picoeca 
        pico_stdlib
        hardware_irq
        cmsis_core
)

# enable usb output, disable uart output
pico_enable_stdio_usb(picoeca 1)
pico_enable_stdio_uart(picoeca 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(picoeca)