cmake_minimum_required(VERSION 3.12)


set(NAME sparkie_fw)
set(PICO_BOARD pico2)
set(PICO_PLATFORM rp2350)

include(pico_sdk_import.cmake)


# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_CXX_ENABLE_RTTI 1)

# Initialize the SDK
pico_sdk_init()

#include Libraries
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include("${FREERTOS_CONFIG_FILE_DIRECTORY}/freertos_config.cmake") 
include(FreeRTOS_Kernel_import.cmake)
add_compile_definitions(
    FREE_RTOS_KERNEL_SMP=1
    portSUPPORT_SMP=1
)

#gy85
add_subdirectory(lib/gy85)

#PicoLed library
include(lib/PicoLED/PicoLed.cmake)

# pico_dht library
add_subdirectory(lib/AhtPico)

# pico pwm library
add_subdirectory(lib/PicoPWM)

# micro ros
include(micro_ros_pico_import.cmake)

#Add main source directory
add_subdirectory(src)


# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)