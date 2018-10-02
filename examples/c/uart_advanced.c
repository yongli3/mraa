/*
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 * Copyright (c) 2018, Linaro Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Example usage: Prints "Hello Mraa!" recursively. Press Ctrl+C to exit
 *
 */

/* standard headers */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

/* mraa header */
#include "mraa/uart.h"

#ifndef FALSE
#define FALSE 0
#define TRUE (!FALSE)
#endif

#define CMD_SETADDR 0
#define CMD_ACK     1
#define CMD_BYPASS  2

typedef struct  __attribute__((packed))
{
    uint32_t header;
    uint8_t command;
    uint8_t srcx;
    uint8_t srcy;
    uint8_t dstx;
    uint8_t dsty;
    uint32_t data;
} UART_PKT;

/* UART port name */
const char* dev_path = "/dev/ttyS0";

volatile sig_atomic_t flag = 1;

// current time in ms
static long long unsigned int current_timestamp()
{
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long unsigned int microseconds = te.tv_sec * 1000LL + te.tv_usec / 1000;
    return microseconds;
}

static void
sig_handler(int signum)
{
    if (signum == SIGINT) {
        fprintf(stdout, "Exiting...\n");
        flag = 0;
    }
}

int
main(int argc, char** argv)
{
    int ret = 0;
    mraa_result_t status = MRAA_SUCCESS;
    mraa_uart_context uart;
    char buffer[32] = "Hello Mraa!";
    UART_PKT tx_pkt, rx_pkt;

    int baudrate = 115200, stopbits = 1, databits = 8;
    mraa_uart_parity_t parity = MRAA_UART_PARITY_NONE;
    unsigned int ctsrts = FALSE, xonxoff = FALSE;
    const char* name = NULL;

    /* install signal handler */
    signal(SIGINT, sig_handler);

    /* initialize mraa for the platform (not needed most of the time) */
    mraa_init();

    //! [Interesting]
    /* initialize uart */
    uart = mraa_uart_init_raw(dev_path);
    if (uart == NULL) {
        fprintf(stderr, "Failed to initialize UART\n");
        return EXIT_FAILURE;
    }

    /* set serial port parameters */
    status =
    mraa_uart_settings(-1, &dev_path, &name, &baudrate, &databits, &stopbits, &parity, &ctsrts, &xonxoff);
    if (status != MRAA_SUCCESS) {
        goto err_exit;
    }

    fprintf(stderr, "baudrate=%d %d\n", baudrate, sizeof(rx_pkt));

    mraa_uart_set_baudrate(uart, 115200);
    mraa_uart_set_flowcontrol(uart, FALSE, FALSE);
    mraa_uart_set_mode(uart, 8, MRAA_UART_PARITY_NONE, 1);

    // start uart Rx/TX
    while (flag) {
        //sprintf(buffer, "%llu", current_timestamp());

         memset(&tx_pkt, 0, sizeof(tx_pkt));
        tx_pkt.header = 0xbeef;
        tx_pkt.command = CMD_SETADDR;
        tx_pkt.srcx = 0xff;
        tx_pkt.srcy = 0xff;
        tx_pkt.dstx = 0x0;
        tx_pkt.dsty = 0x0;
        ret = mraa_uart_write(uart, (char *)&tx_pkt, sizeof(tx_pkt));
        usleep(1000*100);
        //printf("%llu write %d [%s]\n", current_timestamp(), ret, buffer);
        if (mraa_uart_data_available(uart, 1000)) {
            memset(&rx_pkt, 0, sizeof(rx_pkt));
            ret = mraa_uart_read(uart, (char *)&rx_pkt, sizeof(rx_pkt));
            printf("%llu, read %d header=%x\n", current_timestamp(), ret, rx_pkt.header);
        } else {
            printf("%llu no data!\n", current_timestamp());
        }
        usleep(1000*100);
    }

    /* stop uart */
    mraa_uart_stop(uart);

    //! [Interesting]
    /* deinitialize mraa for the platform (not needed most of the time) */
    mraa_deinit();

    return EXIT_SUCCESS;

err_exit:
    mraa_result_print(status);

    /* stop uart */
    mraa_uart_stop(uart);

    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();

    return EXIT_FAILURE;
}
