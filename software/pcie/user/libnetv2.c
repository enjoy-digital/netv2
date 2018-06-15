/*
 * NeTV2 PCIe library
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

#include "netv2.h"
#include "config.h"
#include "csr.h"
#include "flags.h"

#include "libnetv2.h"

#include <bitstream/smpte/291.h>

int64_t get_time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + (ts.tv_nsec / 1000000U);
}

/* ioctl */

uint32_t netv2_readl(int fd, uint32_t addr) {
    struct netv2_ioctl_reg m;
    m.is_write = 0;
    m.addr = addr;
    ioctl(fd, NETV2_IOCTL_REG, &m);
    return m.val;
}

void netv2_writel(int fd, uint32_t addr, uint32_t val) {
    struct netv2_ioctl_reg m;
    m.is_write = 1;
    m.addr = addr;
    m.val = val;
    ioctl(fd, NETV2_IOCTL_REG, &m);
}

void netv2_dma(int fd, uint8_t fill, uint8_t rx_tx_loopback_enable, uint8_t tx_rx_loopback_enable) {
    struct netv2_ioctl_dma m;
    m.fill = fill;
    m.rx_tx_loopback_enable = rx_tx_loopback_enable;
    m.tx_rx_loopback_enable = tx_rx_loopback_enable;
    ioctl(fd, NETV2_IOCTL_DMA, &m);
}

void netv2_dma_writer(int fd, uint8_t enable, int64_t *hw_count, int64_t *sw_count) {
    struct netv2_ioctl_dma_writer m;
    m.enable = enable;
    ioctl(fd, NETV2_IOCTL_DMA_WRITER, &m);
    *hw_count = m.hw_count;
    *sw_count = m.sw_count;
}

void netv2_dma_reader(int fd, uint8_t enable, int64_t *hw_count, int64_t *sw_count) {
    struct netv2_ioctl_dma_reader m;
    m.enable = enable;
    ioctl(fd, NETV2_IOCTL_DMA_READER, &m);
    *hw_count = m.hw_count;
    *sw_count = m.sw_count;
}
