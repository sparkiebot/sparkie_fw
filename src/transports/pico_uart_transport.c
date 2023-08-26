#include "pico_uart_transport.h"
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include <time.h>

#include "../config.hpp"

#include <hardware/dma.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>


#include <FreeRTOS.h>
#include <task.h>

/***
 * Open transport
 * @param transport
 * @return true if ok
 */
bool pico_uart_transport_open(struct uxrCustomTransport * transport){

    uart_init(uart0, 921600);

    gpio_set_function(UART_TX, GPIO_FUNC_UART);
    gpio_set_function(UART_RX, GPIO_FUNC_UART);

	return true;
}

/***
 * Close transport
 * @param transport
 * @return true if OK
 */
bool pico_uart_transport_close(struct uxrCustomTransport * transport){
    uart_deinit(uart0);
    return true;
}

/***
 * Write to the transport
 * @param transport
 * @param buf - Data to write
 * @param len - Length of data
 * @param err - error code
 * @return number of bytes written. <0 if error occurs
 */
size_t pico_uart_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode){
    for (size_t i = 0; i < len; ++i) {
        while (!uart_is_writable(uart0))
            taskYIELD();
        uart_get_hw(uart0)->dr = *buf++;
    }

    return len;
}

/***
 * Read buffer with timeout
 * @param transport
 * @param buf - Data buffer to write to
 * @param len - Max length of buffer
 * @param timeout - timeout in micro Seconds
 * @param err
 * @return returns number of bytes read. < 0 if error occurs
 */
size_t pico_uart_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode){
    size_t i = 0;
    while (uart_is_readable_within_us(uart0, timeout/len)) {
    	buf[i++] = (uint8_t) uart_get_hw(uart0)->dr;
    	
        if (i == len){
    		break;
    	}

    	taskYIELD();
    }
    
    return i;
}