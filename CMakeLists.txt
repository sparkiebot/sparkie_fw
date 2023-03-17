cmake_minimum_required(VERSION 3.12)


set(NAME hubbie_board)

include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

#include Libraries
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") 
include(FreeRTOS_Kernel_import.cmake)
add_compile_definitions(
    FREE_RTOS_KERNEL_SMP=1
    portSUPPORT_SMP=1
)

#icm20689
link_directories("${CMAKE_CURRENT_LIST_DIR}/lib/icm20689pico/build")
include_directories("${CMAKE_CURRENT_LIST_DIR}/lib/icm20689pico/include")

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