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

#define AP_ADDRX    0xff
#define AP_ADDRY    0xff

#define CMD_SETADDR 0
#define CMD_ACK     1
#define CMD_REPORT  2
#define CMD_BYPASS  3

#define UART_PKT_HEADER 0xbeef

typedef struct  __attribute__((packed))
{
    uint16_t header;
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
    int read_size = 0;
    mraa_result_t status = MRAA_SUCCESS;
    mraa_uart_context uart;
    //char buffer[32] = "Hello Mraa!";
    char rxbuf[sizeof(UART_PKT) * 2] = "";
    uint16_t *pbuf = NULL;
    UART_PKT tx_pkt, rx_pkt;
    UART_PKT *prx_pkt = NULL;

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
        printf("%llu set ADDR 0,0\n", current_timestamp());
        memset(&tx_pkt, 0, sizeof(tx_pkt));
        tx_pkt.header = UART_PKT_HEADER;
        tx_pkt.command = CMD_SETADDR;
        tx_pkt.srcx = 0xff;
        tx_pkt.srcy = 0xff;
        tx_pkt.dstx = 0x0;
        tx_pkt.dsty = 0x0;
        tx_pkt.data = current_timestamp();
        ret = mraa_uart_write(uart, (char *)&tx_pkt, sizeof(tx_pkt));
        usleep(1000*50);
        prx_pkt = NULL;
        if (mraa_uart_data_available(uart, 1000)) {
            // wait for ACK
            memset(rxbuf, 0, sizeof(rxbuf));
            ret = mraa_uart_read(uart, rxbuf, sizeof(rxbuf));
            printf("ret=%d %x\n", ret, rxbuf[0]);
            read_size = ret;
            while (read_size < sizeof(UART_PKT)) {
                ret = mraa_uart_read(uart, &rxbuf[read_size], sizeof(rxbuf) - read_size);
                read_size += ret;
                printf("ret=%d %d\n", ret, read_size);
            }
            printf("%llu, read %d %x-%x\n", current_timestamp(), read_size, rxbuf[0], rxbuf[1]);
            pbuf = (uint16_t *) rxbuf;
            if (read_size >= sizeof(UART_PKT)) {
                for (int i = 0; i < read_size; i++) {
                    printf("0x%x\n", rxbuf[i]);
                    pbuf = (uint16_t *) &rxbuf[i];
                    if (*pbuf == UART_PKT_HEADER) {
                        if (i + sizeof(UART_PKT) <= read_size) {
                            printf("FIND@%d!\n", i);
                            prx_pkt = (UART_PKT *)pbuf;
                            break;
                        }
                    }
                }
            }
        } else {
            printf("%llu no data!\n", current_timestamp());
        }

        if (prx_pkt != NULL) {
            memset(&rx_pkt, 0, sizeof(UART_PKT));
            memcpy(&rx_pkt, prx_pkt, sizeof(UART_PKT));
            printf("CMD=%d %d\n", rx_pkt.command, rx_pkt.data);

            switch (rx_pkt.command) {
                case CMD_ACK:
                    // wait for report data; and send out ACK
                    prx_pkt = NULL;
                    while (1) {
                        printf("wait for report!\n");
                        if (mraa_uart_data_available(uart, 1000)) {
                            memset(rxbuf, 0, sizeof(rxbuf));
                            ret = mraa_uart_read(uart, rxbuf, sizeof(rxbuf));
                            printf("ret=%d %x\n", ret, rxbuf[0]);
                            read_size = ret;
                            while (read_size < sizeof(UART_PKT)) {
                                ret = mraa_uart_read(uart, &rxbuf[read_size], sizeof(rxbuf) - read_size);
                                read_size += ret;
                                printf("ret=%d %d\n", ret, read_size);
                            }
                            printf("%llu, read %d %x-%x\n", current_timestamp(), read_size, rxbuf[0], rxbuf[1]);
                            pbuf = (uint16_t *) rxbuf;
                            if (read_size >= sizeof(UART_PKT)) {
                                for (int i = 0; i < read_size; i++) {
                                    printf("0x%x\n", rxbuf[i]);
                                    pbuf = (uint16_t *) &rxbuf[i];
                                    if (*pbuf == UART_PKT_HEADER) {
                                        if (i + sizeof(UART_PKT) <= read_size) {
                                            printf("FIND@%d!\n", i);
                                            prx_pkt = (UART_PKT *)pbuf;
                                            break;
                                        }
                                    }
                                }
                            }
                        }

                        if (prx_pkt != NULL) {
                            // report data
                            memset(&rx_pkt, 0, sizeof(UART_PKT));
                            memcpy(&rx_pkt, prx_pkt, sizeof(UART_PKT));
                            printf("CMD=%d %d\n", rx_pkt.command, rx_pkt.data);
                            switch (rx_pkt.command) {
                                case CMD_REPORT:
                                    printf("Get REPORT: %d,%d->%d,%d %d\n", rx_pkt.srcx, rx_pkt.srcy, rx_pkt.dstx, rx_pkt.dsty, rx_pkt.data);
                                    // send out ACK
                                    memset(&tx_pkt, 0, sizeof(UART_PKT));
                                    tx_pkt.header = UART_PKT_HEADER;
                                    tx_pkt.command = CMD_ACK;
                                    tx_pkt.srcx = AP_ADDRX;
                                    tx_pkt.srcy = AP_ADDRY;
                                    tx_pkt.dstx = rx_pkt.srcx;
                                    tx_pkt.dsty = rx_pkt.srcy;
                                    ret = mraa_uart_write(uart, (char *)&tx_pkt, sizeof(tx_pkt));
                                    break;
                                default:
                                    break;
                            }
                        }
                        usleep(1000*100);
                    }
                    
                    break;
                default:
                    printf("Get incorrect CMD %d\n", rx_pkt.command);
                    break;
            }
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
