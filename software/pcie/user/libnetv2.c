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

void netv2_reload(int fd) {
    struct netv2_ioctl_icap m;
	m.addr = 0x4;
	m.data = 0xf;
    ioctl(fd, NETV2_IOCTL_ICAP, &m);
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
/* flash */

static uint64_t flash_spi(int fd, int tx_len, uint8_t cmd,
                          uint32_t tx_data)
{
    struct netv2_ioctl_flash m;
    m.tx_len = tx_len;
    m.tx_data = tx_data | ((uint64_t)cmd << 32);
    if (ioctl(fd, NETV2_IOCTL_FLASH, &m) < 0) {
        perror("NETV2_IOCTL_FLASH");
        exit(1);
    }
    return m.rx_data;
}

static uint32_t flash_read_id(int fd)
{
    return flash_spi(fd, 32, FLASH_READ_ID, 0) & 0xffffff;
}

static void flash_write_enable(int fd)
{
    flash_spi(fd, 8, FLASH_WREN, 0);
}

static void flash_write_disable(int fd)
{
    flash_spi(fd, 8, FLASH_WRDI, 0);
}

static uint8_t flash_read_status(int fd)
{
    return flash_spi(fd, 16, FLASH_RDSR, 0) & 0xff;
}

static __attribute__((unused)) void flash_write_status(int fd, uint8_t value)
{
    flash_spi(fd, 16, FLASH_WRSR, value << 24);
}

static void flash_erase_sector(int fd, uint32_t addr)
{
    flash_spi(fd, 32, FLASH_SE, addr << 8);
}

static __attribute__((unused)) uint8_t flash_read_sector_lock(int fd, uint32_t addr)
{
    return flash_spi(fd, 40, FLASH_WRSR, addr << 8) & 0xff;
}

static __attribute__((unused)) void flash_write_sector_lock(int fd, uint32_t addr, uint8_t byte)
{
    flash_spi(fd, 40, FLASH_WRSR, (addr << 8) | byte);
}

static void flash_write(int fd, uint32_t addr, uint8_t byte)
{
    flash_spi(fd, 40, FLASH_PP, (addr << 8) | byte);
}

uint8_t netv2_flash_read(int fd, uint32_t addr)
{
    return flash_spi(fd, 40, FLASH_READ, addr << 8) & 0xff;
}

int netv2_flash_get_erase_block_size(int fd)
{
    return FLASH_SECTOR_SIZE;
}

void netv2_flash_write(int fd,
                     const uint8_t *buf, uint32_t base, uint32_t size,
                     void (*progress_cb)(void *opaque, const char *fmt, ...),
                     void *opaque)
{
    int i;

    /* dummy command because in some case the first erase does not
       work. */
    flash_read_id(fd);

    /* erase */
    for(i = 0; i < size; i += FLASH_SECTOR_SIZE) {
        if (progress_cb) {
            progress_cb(opaque, "Erasing %08x\r", base + i);
        }
        flash_write_enable(fd);
        flash_erase_sector(fd, base + i);
        while (flash_read_status(fd) & FLASH_WIP) {
            usleep(10 * 1000);
        }
    }
    if (progress_cb) {
        progress_cb(opaque, "\n");
    }
    flash_write_disable(fd);

    /* program */
    for(i = 0; i < size; i++) {
        if (progress_cb && (i % FLASH_SECTOR_SIZE) == 0) {
            progress_cb(opaque, "Writing %08x\r", base + i);
        }
        while (flash_read_status(fd) & FLASH_WIP) {
            usleep(10 * 1000);
        }
        flash_write_enable(fd);
        flash_write(fd, base + i, buf[i]);
        flash_write_disable(fd);
    }
    if (progress_cb) {
        progress_cb(opaque, "\n");
    }
}
