/*
 * NeTV2 PCIe driver
 *
 * Copyright (C) 2018 / LambdaConcept / ramtin@lambdaconcept.com
 * Copyright (C) 2018 / LambdaConcept / po@lambdaconcept.com
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

#ifndef _LINUX_NETV2_H
#define _LINUX_NETV2_H

#include <linux/types.h>

struct netv2_ioctl_reg {
    uint32_t addr;
    uint32_t val;
    uint8_t is_write;
};

struct netv2_ioctl_dma {
    uint8_t fill;
    uint8_t tx_rx_loopback_enable;
    uint8_t rx_tx_loopback_enable;
};

struct netv2_ioctl_dma_writer {
    uint8_t enable;
    int64_t hw_count;
    int64_t sw_count;
};

struct netv2_ioctl_dma_reader {
    uint8_t enable;
    int64_t hw_count;
    int64_t sw_count;
};

#define NETV2_IOCTL 'S'

#define NETV2_IOCTL_REG               _IOWR(NETV2_IOCTL,  0, struct netv2_ioctl_reg)

#define NETV2_IOCTL_DMA               _IOW(NETV2_IOCTL,  10, struct netv2_ioctl_dma)
#define NETV2_IOCTL_DMA_WRITER        _IOWR(NETV2_IOCTL, 11, struct netv2_ioctl_dma_writer)
#define NETV2_IOCTL_DMA_READER        _IOWR(NETV2_IOCTL, 12, struct netv2_ioctl_dma_reader)

#endif /* _LINUX_NETV2_H */
