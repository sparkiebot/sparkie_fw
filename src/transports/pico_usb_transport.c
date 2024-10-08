#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include <time.h>
#include <tusb.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>


#include <FreeRTOS.h>
#include <task.h>

/***
 * Sleep via FreeRTOS sleep
 * @param us
 */
void usleep(uint64_t us){
	sleep_us(us);
}

/***
 * Get Time since boot
 * @param unused
 * @param tp
 * @return
 */
int clock_gettime(clockid_t unused, struct timespec *tp){
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

/***
 * Open transport
 * @param transport
 * @return true if ok
 */
bool pico_usb_transport_open(struct uxrCustomTransport * transport){
    return true;
}

/***
 * Close transport
 * @param transport
 * @return true if OK
 */
bool pico_usb_transport_close(struct uxrCustomTransport * transport){
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

size_t pico_usb_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    stdio_usb.out_chars(buf, len);
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

size_t pico_usb_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    uint64_t until_time_us = time_us_64() + timeout;

    size_t read = 0;
    while (time_us_64() < until_time_us)
    {
        read = stdio_usb.in_chars(buf, len);
        if (read != 0)
        {
            return read;
        }

        taskYIELD();
    }

    return 0;
}