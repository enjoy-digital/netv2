/*
 * NeTV2 PCIe util
 *
 * Copyright (C) 2018 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <time.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include "netv2.h"
#include "config.h"
#include "csr.h"
#include "flags.h"

#include "libnetv2.h"

static char netv2_device[1024];
static int netv2_device_num;

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

/* info */

static void info(void)
{
    int fd;
    int i;
    unsigned char fpga_identification[256];

    fd = open(netv2_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    for(i=0; i<256; i++)
        fpga_identification[i] = netv2_readl(fd, CSR_IDENTIFIER_MEM_BASE + 4*i);
    printf("FPGA identification: %s\n", fpga_identification);
    printf("FPGA dna: 0x%08x%08x\n",
        netv2_readl(fd, CSR_DNA_ID_ADDR + 4*0),
        netv2_readl(fd, CSR_DNA_ID_ADDR + 4*1)
    );
    printf("FPGA temperature: %0.1f °C\n",
           (double)netv2_readl(fd, CSR_XADC_TEMPERATURE_ADDR) * 503.975/4096 - 273.15);
    printf("FPGA vccint: %0.2f V\n",
           (double)netv2_readl(fd, CSR_XADC_VCCINT_ADDR) / 4096 * 3);
    printf("FPGA vccaux: %0.2f V\n",
           (double)netv2_readl(fd, CSR_XADC_VCCAUX_ADDR) / 4096 * 3);
    printf("FPGA vccbram: %0.2f V\n",
           (double)netv2_readl(fd, CSR_XADC_VCCBRAM_ADDR) / 4096 * 3);

    close(fd);
}

/* dma */

static inline char seed_to_data(uint32_t seed)
{
#if 0
    return seed * 69069 + 1;
#else
    return seed;
#endif
}

static void write_pn_data(char *buf, int count, uint32_t *pseed)
{
    int i;
    uint32_t seed;

    seed = *pseed;
    for(i = 0; i < count; i++) {
        buf[i] = seed_to_data(seed);
        seed++;
    }
    *pseed = seed;
}

//#define DEBUG_DMA

static int check_pn_data(char *buf, int count, uint32_t *pseed)
{
    int i, errors;
    uint32_t seed;

#ifdef DEBUG_DMA
    static uint32_t errors_report = 0;
#endif

    errors = 0;
    seed = *pseed;
    for(i = 0; i < count; i++) {
        if (buf[i] != seed_to_data(seed)) {
            errors++;
#ifdef DEBUG_DMA
            if (errors_report < 128) {
                printf("error: 0x%02x (0x%02x)\n", (unsigned char) buf[i], (unsigned char) seed_to_data(seed));
                errors_report++;
            }
#endif
        }
        seed++;
    }
    *pseed = seed;
    return errors;
}

static void dma(void)
{
    struct pollfd fds;
    int ret;
    int i;
    ssize_t len;
    ssize_t total_len;

    int64_t reader_hw_count, reader_sw_count, reader_sw_count_last;
    int64_t writer_hw_count, writer_sw_count;

    int64_t duration;
    int64_t last_time;

    uint32_t seed_wr;
    uint32_t seed_rd;

    uint32_t errors;

    char *buf_rd, *buf_wr;
    uint32_t buf_wr_offset;

    buf_rd = malloc(DMA_BUFFER_SIZE * DMA_BUFFER_COUNT);
    buf_wr = malloc(DMA_BUFFER_SIZE * DMA_BUFFER_COUNT * 2);
    buf_wr_offset = 0;

    errors = 0;
    seed_wr = 0;
    seed_rd = 0;

    write_pn_data(buf_wr, DMA_BUFFER_SIZE * DMA_BUFFER_COUNT, &seed_wr);
    memcpy(buf_wr + DMA_BUFFER_SIZE * DMA_BUFFER_COUNT, buf_wr,
        DMA_BUFFER_SIZE * DMA_BUFFER_COUNT);
    memset(buf_rd, 0, DMA_BUFFER_SIZE * DMA_BUFFER_COUNT);
#if 0
    buf_wr[0] = 0x5a; /* error injection */
#endif

    fds.fd = open(netv2_device, O_RDWR | O_CLOEXEC);
    fds.events = POLLIN | POLLOUT;
    if (fds.fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* enable dma tx rx loopback*/
    netv2_dma(fds.fd, 0, 0, 1);

    /* test loop */
    i = 0;
    total_len = 0;
    reader_hw_count = 0;
    reader_sw_count = 0;
    reader_sw_count_last = 0;
    writer_hw_count = 0;
    writer_sw_count = 0;
    last_time = get_time_ms();
    for (;;) {
        /* exit loop on key pressed */
        if (kbhit())
            break;

        /* set / get dma */
        netv2_dma_writer(fds.fd, 1, &writer_hw_count, &writer_sw_count);
        netv2_dma_reader(fds.fd, 1, &reader_hw_count, &reader_sw_count);

        /* polling */
        ret = poll(&fds, 1, 100);
        if (ret <=  0) {
            continue;
        }

        /* read event */
        if (fds.revents & POLLIN) {
            len = read(fds.fd, buf_rd, DMA_BUFFER_SIZE * DMA_BUFFER_COUNT);
            if(len >= 0) {
                total_len += len;
                /* start checking after one dma loop */
                if (total_len > DMA_BUFFER_SIZE * DMA_BUFFER_COUNT)
                    errors += check_pn_data(buf_rd, len, &seed_rd);
            }
        }

        /* write event */
        if (fds.revents & POLLOUT) {
            len = write(fds.fd, buf_wr + buf_wr_offset, DMA_BUFFER_SIZE * DMA_BUFFER_COUNT);
            switch (len) {
                case -1:
                    perror("write");
                    break;
                default:
                    buf_wr_offset += len;
                    buf_wr_offset = buf_wr_offset % (DMA_BUFFER_SIZE * DMA_BUFFER_COUNT);
                    break;
            }
        }

        /* statistics */
        duration = get_time_ms() - last_time;
        if (duration > 200) {
            if(i%10 == 0)
                printf("\e[1mDMA_SPEED(Gbps) TX_BUFFERS RX_BUFFERS   DIFF  SYNCED  ERRORS\e[0m\n");
            i++;
            printf("%15.1f %10" PRIu64 " %10" PRIu64 " %6" PRIu64 " %7u %7u\n",
                    (double)(reader_sw_count - reader_sw_count_last) * DMA_BUFFER_SIZE * 8 / ((double)duration * 1e6),
                    reader_sw_count,
                    writer_sw_count,
                    reader_sw_count - writer_sw_count,
                    abs(reader_sw_count - writer_sw_count) < DMA_BUFFER_COUNT,
                    errors);

            last_time = get_time_ms();
            reader_sw_count_last = reader_hw_count;
        }
    }

    netv2_dma_reader(fds.fd, 0, &reader_hw_count, &reader_sw_count);
    netv2_dma_writer(fds.fd, 0, &writer_hw_count, &writer_sw_count);

    free(buf_rd);
    free(buf_wr);

    close(fds.fd);
}


static void help(void)
{
    printf("NeTV2 PCIe board utilities\n"
           "usage: netv2_util [options] cmd [args...]\n"
           "\n"
           "options:\n"
           "-h                               Help\n"
           "-c device_num                    Select the device (default = 0)\n"
           "\n"
           "available commands:\n"
           "info                             Board information\n"
           "dma                              Test DMA (loopback in FPGA)\n"
           );
    exit(1);
}

int main(int argc, char **argv)
{
    const char *cmd;
    int c;

    netv2_device_num = 0;
    for(;;) {
        c = getopt(argc, argv, "hc:");
        if (c == -1)
            break;
        switch(c) {
        case 'h':
            help();
            break;
        case 'c':
            netv2_device_num = atoi(optarg);
            break;
        default:
            exit(1);
        }
    }

    if (optind >= argc)
        help();

    snprintf(netv2_device, sizeof(netv2_device), "/dev/netv2%d", netv2_device_num);

    cmd = argv[optind++];

    if (!strcmp(cmd, "info"))
        info();
    else if (!strcmp(cmd, "dma"))
        dma();
    else
show_help:
        help();

    return 0;
}
