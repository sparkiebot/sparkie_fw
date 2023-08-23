file(GLOB_RECURSE SRC "*")


add_executable(${NAME}
    ${SRC}
)

pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/misc/us_echo.pio)
pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/misc/us_trig.pio)


target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../lib/FreeRTOS-Kernel/include
    ${CMAKE_CURRENT_LIST_DIR}/../lib/icm20689pico/include
     )


# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	 pico_stdlib
     pico_multicore
     hardware_i2c
     hardware_watchdog
     hardware_pwm
     hardware_adc
     hardware_pio
     FreeRTOS-Kernel-Heap4
     microros
     icm20689pico
     PicoLed
     PicoPWM
     dht
	)
  
target_compile_definitions(${NAME} PRIVATE
    FREE_RTOS_KERNEL_SMP=1
    portSUPPORT_SMP=1
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)

# micro ros formatting requirements
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

#add_compile_definitions(PICO_DEFAULT_UART_TX_PIN=16)
#add_compile_definitions(PICO_DEFAULT_UART_RX_PIN=17)

# Usb settings
add_compile_definitions(USBD_MANUFACTURER="FortiSix")
add_compile_definitions(USBD_PRODUCT="Sparkie Board")


# Adding this compile option makes the board boot correctly when connected.
# https://github.com/raspberrypi/pico-sdk/pull/457
add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64) 


add_compile_definitions(DEBUG)