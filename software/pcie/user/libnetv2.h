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

#ifndef NETV2_LIB_H
#define NETV2_LIB_H

int64_t get_time_ms(void);

/* ioctl */

uint32_t netv2_readl(int fd, uint32_t addr);
void netv2_writel(int fd, uint32_t addr, uint32_t val);
void netv2_reload(int fd);
void netv2_dma(int fd, uint8_t fill, uint8_t rx_tx_loopback_enable, uint8_t tx_rx_loopback_enable);
void netv2_dma_reader(int fd, uint8_t enable, int64_t *hw_count, int64_t *sw_count);
void netv2_dma_writer(int fd, uint8_t enable, int64_t *hw_count, int64_t *sw_count);

/* flash */

#define FLASH_READ_ID 0x9E
#define FLASH_READ    0x03
#define FLASH_WREN    0x06
#define FLASH_WRDI    0x04
#define FLASH_PP      0x02
#define FLASH_SE      0xD8
#define FLASH_BE      0xC7
#define FLASH_RDSR    0x05
#define FLASH_WRSR    0x01
/* status */
#define FLASH_WIP     0x01

#define FLASH_SECTOR_SIZE (1 << 16)

uint8_t netv2_flash_read(int fd, uint32_t addr);
int netv2_flash_get_erase_block_size(int fd);
void netv2_flash_write(int fd,
                     const uint8_t *buf, uint32_t base, uint32_t size,
                     void (*progress_cb)(void *opaque, const char *fmt, ...),
                     void *opaque);

#endif /* NETV2_LIB_H */
