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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mmtimer.h>
#include <linux/miscdevice.h>
#include <linux/posix-timers.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/log2.h>

#include "netv2.h"
#include "csr.h"
#include "config.h"
#include "flags.h"

//#define DEBUG_CSR
//#define DEBUG_MSI
//#define DEBUG_POLL

#define NETV2_NAME "netv2"
#define NETV2_MINOR_COUNT 32

struct netv2_dma_chan {
    uint32_t base;
    uint32_t writer_interrupt;
    uint32_t reader_interrupt;
    dma_addr_t reader_handle[DMA_BUFFER_COUNT];
    dma_addr_t writer_handle[DMA_BUFFER_COUNT];
    uint32_t *reader_addr[DMA_BUFFER_COUNT];
    uint32_t *writer_addr[DMA_BUFFER_COUNT];
    int64_t reader_hw_count;
    int64_t reader_hw_count_last;
    int64_t reader_sw_count;
    int64_t writer_hw_count;
    int64_t writer_hw_count_last;
    int64_t writer_sw_count;
    uint8_t writer_enable;
    uint8_t reader_enable;
};

struct netv2_chan {
    struct netv2_device *netv2_dev;
    struct netv2_dma_chan dma;
    struct cdev *cdev;
    uint32_t block_size;
    wait_queue_head_t wait_rd; /* to wait for an ongoing read */
    wait_queue_head_t wait_wr; /* to wait for an ongoing write */

    int index;
    int minor;
};

struct netv2_device {
    struct pci_dev  *dev;
    resource_size_t bar0_size;
    phys_addr_t bar0_phys_addr;
    uint8_t *bar0_addr; /* virtual address of BAR0 */
    struct netv2_chan chan[DMA_CHANNEL_COUNT];
    struct list_head list;
    spinlock_t reg_lock;
    int minor_base;
};

static LIST_HEAD(netv2_list);

static int netv2_major;
static int netv2_minor_idx;
static struct class* netv2_class;
static dev_t netv2_dev_t;

static inline uint32_t netv2_readl(struct netv2_device  *s, uint32_t addr)
{
    uint32_t val;
    val = readl(s->bar0_addr + addr);
#ifdef DEBUG_CSR
    printk("csr_read: 0x%08x @ 0x%08x", val, addr);
#endif
    return val;
}

static inline void netv2_writel(struct netv2_device *s, uint32_t addr, uint32_t val)
{
#ifdef DEBUG_CSR
    printk("csr_write: 0x%08x @ 0x%08x", val, addr);
#endif
    return writel(val, s->bar0_addr + addr);
}

static void netv2_enable_interrupt(struct netv2_device *s, int irq_num)
{
    uint32_t v;
    v = netv2_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v |= (1 << irq_num);
    netv2_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void netv2_disable_interrupt(struct netv2_device *s, int irq_num)
{
    uint32_t v;
    v = netv2_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v &= ~(1 << irq_num);
    netv2_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static int netv2_dma_free(struct netv2_device *s)
{
    int i, j;
    struct netv2_dma_chan *dmachan;

    if(!s)
        return -ENODEV;

    /* for each dma channel */
    for(i = 0; i < DMA_CHANNEL_COUNT; i++) {
        dmachan = &s->chan[i].dma;
        /* for each dma buffer */
        for(j = 0; j < DMA_BUFFER_COUNT; j++) {
            /* free rd */
            if(dmachan->reader_addr[j]) {
                dma_free_coherent(&s->dev->dev, DMA_BUFFER_SIZE,
                                  dmachan->reader_addr[j],
                                  dmachan->reader_handle[j]);
                dmachan->reader_addr[j] = NULL;
                dmachan->reader_handle[j] = 0;
            }
            /* free wr */
            if(dmachan->writer_addr[j]) {
                dma_free_coherent(&s->dev->dev, DMA_BUFFER_SIZE,
                                  dmachan->writer_addr[j],
                                  dmachan->writer_handle[j]);
                dmachan->writer_addr[j] = NULL;
                dmachan->writer_handle[j] = 0;
            }
        }
    }

    return 0;
}

static int netv2_dma_init(struct netv2_device *s)
{

    int i, j;
    struct netv2_dma_chan *dmachan;
    int ret;

    if(!s)
        return -ENODEV;

    /* for each dma channel */
    for(i = 0; i < DMA_CHANNEL_COUNT; i++) {
        dmachan = &s->chan[i].dma;
        /* for each dma buffer */
        for(j = 0; j < DMA_BUFFER_COUNT; j++) {
            /* allocate rd */
            dmachan->reader_addr[j] = dma_zalloc_coherent(
                &s->dev->dev,
                DMA_BUFFER_SIZE,
                &dmachan->reader_handle[j],
                GFP_KERNEL | GFP_DMA32);
            /* allocate wr */
            dmachan->writer_addr[j] = dma_zalloc_coherent(
                &s->dev->dev,
                DMA_BUFFER_SIZE,
                &dmachan->writer_handle[j],
                GFP_KERNEL | GFP_DMA32);
            /* check */
            if(!dmachan->writer_addr[j]
               || !dmachan->reader_addr[j]) {
                printk(KERN_ERR NETV2_NAME " Failed to allocate dma buffers\n");
                ret = -ENOMEM;
                goto fail;
            }
        }
    }

    return 0;
fail:
    netv2_dma_free(s);
    return ret;
}

static int netv2_dma_fill(struct netv2_device *s, int chan_num, int rx_tx_loopback)
{
    struct netv2_dma_chan *dmachan;
    int ret = 0;
    int i;

    if(!s) {
        return -ENODEV;
    }

    dmachan = &s->chan[chan_num].dma;

    /* fill dma writer descriptors */
    if (dmachan->writer_enable == 0) { /* fill only if disabled */
        netv2_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
        netv2_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
        netv2_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0);
        for(i = 0; i < DMA_BUFFER_COUNT; i++) {
            netv2_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET,
#ifndef DMA_BUFFER_ALIGNED
                       DMA_LAST_DISABLE |
#endif
                       (!(i%DMA_BUFFER_PER_IRQ == 0)) * DMA_IRQ_DISABLE | /* generate an msi */
                       DMA_BUFFER_SIZE);                                  /* every n buffers */
            netv2_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET + 4,
                       dmachan->writer_handle[i]);
            netv2_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_WE_OFFSET, 1);
        }
        netv2_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 1);
    }


    /* fill dma reader descriptors */
    if (dmachan->reader_enable == 0) { /* fill only if disabled */
        netv2_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET , 0);
        netv2_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
        netv2_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0);
        for(i = 0; i < DMA_BUFFER_COUNT; i++) {
            netv2_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET,
#ifndef DMA_BUFFER_ALIGNED
                       DMA_LAST_DISABLE |
#endif
                       (!(i%DMA_BUFFER_PER_IRQ == 0)) * DMA_IRQ_DISABLE | /* generate an msi */
                       DMA_BUFFER_SIZE);                                  /* every n buffers */
            if (rx_tx_loopback)
                netv2_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET + 4,
                    dmachan->writer_handle[i]);
            else
                netv2_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET + 4,
                    dmachan->reader_handle[i]);
            netv2_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_WE_OFFSET, 1);
        }
        netv2_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 1);
    }

    return ret;
}

void netv2_stop_dma(struct netv2_device *s)
{
    struct netv2_dma_chan *dmachan;
    int i;

    for(i = 0; i < DMA_CHANNEL_COUNT; i++) {
        dmachan = &s->chan[i].dma;
        netv2_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
        netv2_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    }
}

static irqreturn_t netv2_interrupt(int irq, void *data)
{
    struct netv2_device *s = (struct netv2_device*) data;
    struct netv2_chan *chan;
    uint32_t loop_status;
    uint32_t clear_mask, irq_vector;
    int i;

    irq_vector = netv2_readl(s, CSR_PCIE_MSI_VECTOR_ADDR);
#ifdef DEBUG_MSI
    printk(KERN_INFO NETV2_NAME " MSI: 0x%x 0x%x\n", irq_vector,
           netv2_readl(s, CSR_PCIE_MSI_ENABLE_ADDR));
#endif
    clear_mask = 0;

    for(i = 0; i < DMA_CHANNEL_COUNT; i++) {
        chan = &s->chan[i];
        if(irq_vector & (1 << chan->dma.reader_interrupt)) {
            loop_status = netv2_readl(s, chan->dma.base +
                PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET);
            chan->dma.reader_hw_count &= ((~(DMA_BUFFER_COUNT - 1) << 16) & 0xffffffffffff0000);
            chan->dma.reader_hw_count |= (loop_status >> 16) * DMA_BUFFER_COUNT + (loop_status & 0xffff);
            if (chan->dma.reader_hw_count_last > chan->dma.reader_hw_count)
                chan->dma.reader_hw_count += (1 << (ilog2(DMA_BUFFER_COUNT) + 16));
            chan->dma.reader_hw_count_last = chan->dma.reader_hw_count;
#ifdef DEBUG_MSI
            printk(KERN_INFO NETV2_NAME " MSI DMA%d Reader buf: %d\n", i, chan->dma.reader_hw_count);
#endif
            wake_up_interruptible(&chan->wait_wr);
            clear_mask |= (1 << chan->dma.reader_interrupt);
        }
        if(irq_vector & (1 << chan->dma.writer_interrupt)) {
            loop_status = netv2_readl(s, chan->dma.base +
                PCIE_DMA_WRITER_TABLE_LOOP_STATUS_OFFSET);
            chan->dma.writer_hw_count &= ((~(DMA_BUFFER_COUNT - 1) << 16) & 0xffffffffffff0000);
            chan->dma.writer_hw_count |= (loop_status >> 16) * DMA_BUFFER_COUNT + (loop_status & 0xffff);
            if (chan->dma.writer_hw_count_last > chan->dma.writer_hw_count)
                chan->dma.writer_hw_count += (1 << (ilog2(DMA_BUFFER_COUNT) + 16));
            chan->dma.writer_hw_count_last = chan->dma.writer_hw_count;
#ifdef DEBUG_MSI
            printk(KERN_INFO NETV2_NAME " MSI DMA%d Writer buf: %d\n", i, chan->dma.writer_hw_count);
#endif
            wake_up_interruptible(&chan->wait_rd);
            clear_mask |= (1 << chan->dma.writer_interrupt);
        }
    }

    netv2_writel(s, CSR_PCIE_MSI_CLEAR_ADDR, clear_mask);

    return IRQ_HANDLED;
}

static int netv2_open(struct inode *inode, struct file *file)
{
    int subminor;
    struct netv2_device *netv2;
    struct netv2_chan *chan;

    subminor = iminor(inode);

    list_for_each_entry(netv2, &netv2_list, list) {
        if( (netv2->minor_base <= subminor) &&
            (subminor < netv2->minor_base + DMA_CHANNEL_COUNT) ) {

            chan = &netv2->chan[subminor - netv2->minor_base];
            file->private_data = chan;

            /* fill dma desciptors */
            netv2_dma_fill(netv2, chan->index, 0);

            /* enable interrupts */
            spin_lock(&chan->netv2_dev->reg_lock);
            netv2_enable_interrupt(netv2, chan->dma.writer_interrupt);
            netv2_enable_interrupt(netv2, chan->dma.reader_interrupt);
            spin_unlock(&chan->netv2_dev->reg_lock);

            if (chan->dma.reader_enable == 0) { /* clear only if disabled */
                chan->dma.reader_hw_count = 0;
                chan->dma.reader_hw_count_last = 0;
                chan->dma.reader_sw_count = 0;
            }

            if (chan->dma.writer_enable == 0) { /* clear only if disabled */
                chan->dma.writer_hw_count = 0;
                chan->dma.writer_hw_count_last = 0;
                chan->dma.writer_sw_count = 0;
            }

            return 0;
        }
    }
    return -1;
}

static int netv2_release(struct inode *inode, struct file *file)
{
    struct netv2_chan *chan;

    chan = file->private_data;

    /* disable interrupts */
    spin_lock(&chan->netv2_dev->reg_lock);
    if (chan->dma.writer_enable == 0) /* clear only if disabled */
        netv2_disable_interrupt(chan->netv2_dev, chan->dma.writer_interrupt);
    if (chan->dma.reader_enable == 0) /* clear only if disabled */
        netv2_disable_interrupt(chan->netv2_dev, chan->dma.reader_interrupt);
    spin_unlock(&chan->netv2_dev->reg_lock);

    /* stop dma reader */
    if (chan->dma.reader_enable == 0) { /* stop only if disabled */
        netv2_writel(chan->netv2_dev, chan->dma.base +
            PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0);
        netv2_writel(chan->netv2_dev, chan->dma.base +
            PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
        udelay(1000);
        netv2_writel(chan->netv2_dev, chan->dma.base +
            PCIE_DMA_READER_ENABLE_OFFSET, 0);
        netv2_writel(chan->netv2_dev, chan->dma.base +
            PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
    }

    /* stop dma writer */
    if (chan->dma.writer_enable == 0) { /* stop only if disabled */
        netv2_writel(chan->netv2_dev, chan->dma.base +
            PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0);
        netv2_writel(chan->netv2_dev, chan->dma.base +
            PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
        udelay(1000);
        netv2_writel(chan->netv2_dev, chan->dma.base +
            PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
        netv2_writel(chan->netv2_dev, chan->dma.base +
            PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
    }

    return 0;
}

static ssize_t netv2_read(struct file *file, char __user *data, size_t size, loff_t *offset)
{
    struct netv2_chan *chan;
    size_t len;
    int i, ret;
    int overflows;

    chan = (struct netv2_chan *)file->private_data;

    /* if nothing has changed, we block
     * TODO: check if O_NONBLOCK
     */
    ret = wait_event_interruptible(chan->wait_rd,
        (chan->dma.writer_hw_count - chan->dma.writer_sw_count) >= 0);

    if(ret < 0)
        return ret;

    i = 0;
    overflows = 0;
    len = size;
    while (len >= DMA_BUFFER_SIZE) {
        if ((chan->dma.writer_hw_count - chan->dma.writer_sw_count) >= 0) {
            if ((chan->dma.writer_hw_count - chan->dma.writer_sw_count) > DMA_BUFFER_COUNT/2) {
                overflows++;
            } else {
                ret = copy_to_user(data + (chan->block_size * i),
                           chan->dma.writer_addr[chan->dma.writer_sw_count%DMA_BUFFER_COUNT],
                           DMA_BUFFER_SIZE);
                if(ret)
                    return -EFAULT;
            }
            len -= DMA_BUFFER_SIZE;
            chan->dma.writer_sw_count += 1;
            i++;
        } else {
            break;
        }
    }

    if (overflows)
        printk(KERN_INFO NETV2_NAME " Reading too late, %d buffers lost\n", overflows);

    return size - len;
}

static ssize_t netv2_write(struct file *file, const char __user *data, size_t size, loff_t *offset)
{
    struct netv2_chan *chan;
    size_t len;
    int i, ret;
    int underflows;

    chan = (struct netv2_chan *)file->private_data;

    /* if buffer available, we block
     * TODO: check if O_NONBLOCK
     */
    ret = wait_event_interruptible(chan->wait_wr,
        (chan->dma.reader_sw_count - chan->dma.reader_hw_count) < DMA_BUFFER_COUNT/2);

    i = 0;
    underflows = 0;
    len = size;
    while (len >= DMA_BUFFER_SIZE) {
        if ((chan->dma.reader_sw_count - chan->dma.reader_hw_count) < DMA_BUFFER_COUNT/2) {
            if ((chan->dma.reader_sw_count - chan->dma.reader_hw_count) < 0) {
                underflows++;
            } else {
                ret = copy_from_user(chan->dma.reader_addr[chan->dma.reader_sw_count%DMA_BUFFER_COUNT],
                                     data + (chan->block_size * i), DMA_BUFFER_SIZE);
                if(ret)
                    return -EFAULT;
            }
            len -= DMA_BUFFER_SIZE;
            chan->dma.reader_sw_count += 1;
            i++;
        } else {
            break;
        }
    }

    if (underflows)
        printk(KERN_INFO NETV2_NAME " Writing too late, %d buffers lost\n", underflows);

    return size - len;
}

static unsigned int netv2_poll(struct file *file, poll_table *wait)
{
    struct netv2_chan *chan;
    unsigned int mask = 0;

    chan = (struct netv2_chan *)file->private_data;
    poll_wait(file, &chan->wait_rd, wait);
    poll_wait(file, &chan->wait_wr, wait);

#ifdef DEBUG_POLL
    printk(KERN_INFO NETV2_NAME " poll: writer hw_count: %10lld / sw_count %10lld \n",
        chan->dma.writer_hw_count, chan->dma.writer_sw_count);
    printk(KERN_INFO NETV2_NAME " poll: reader hw_count: %10lld / sw_count %10lld \n",
        chan->dma.reader_hw_count, chan->dma.reader_sw_count);
#endif

    if((chan->dma.writer_hw_count - chan->dma.writer_sw_count) >= 0)
        mask |= POLLIN | POLLRDNORM;

    if ((chan->dma.reader_sw_count - chan->dma.reader_hw_count) < DMA_BUFFER_COUNT/2)
        mask |= POLLOUT | POLLWRNORM;

    return mask;
}

/* SPI */

static int netv2_flash_spi(struct netv2_device *s, struct netv2_ioctl_flash *m)
{
    if (m->tx_len < 8 || m->tx_len > 40)
        return -EINVAL;

    netv2_writel(s, CSR_FLASH_SPI_MOSI_ADDR, m->tx_data >> 32);
    netv2_writel(s, CSR_FLASH_SPI_MOSI_ADDR + 4, m->tx_data);
    netv2_writel(s, CSR_FLASH_SPI_CTRL_ADDR,
               SPI_CTRL_START | (m->tx_len * SPI_CTRL_LENGTH));
    udelay(16);
    while ((netv2_readl(s, CSR_FLASH_SPI_STATUS_ADDR) & SPI_STATUS_DONE) == 0)
        udelay(1);
    m->rx_data = ((uint64_t)netv2_readl(s, CSR_FLASH_SPI_MISO_ADDR) << 32) |
        netv2_readl(s, CSR_FLASH_SPI_MISO_ADDR + 4);
    return 0;
}

static long netv2_ioctl(struct file *file, unsigned int cmd,
                      unsigned long arg)
{
    struct netv2_chan *chan;
    long ret = 0;

    chan = (struct netv2_chan *)file->private_data;

    switch(cmd) {
    case NETV2_IOCTL_REG:
        {
            struct netv2_ioctl_reg m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }
            if (m.is_write)
                netv2_writel(chan->netv2_dev, m.addr, m.val);
            else
                m.val = netv2_readl(chan->netv2_dev, m.addr);

            if (copy_to_user((void *)arg, &m, sizeof(m))) {
                ret = -EFAULT;
                break;
            }
        }
        break;
    case NETV2_IOCTL_FLASH:
        {
            struct netv2_ioctl_flash m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }
            ret = netv2_flash_spi(chan->netv2_dev, &m);
            if (ret == 0) {
                if (copy_to_user((void *)arg, &m, sizeof(m))) {
                    ret = -EFAULT;
                    break;
                }
            }
        }
        break;
    case NETV2_IOCTL_ICAP:
        {
            struct netv2_ioctl_icap m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            netv2_writel(chan->netv2_dev, CSR_ICAP_ADDR_ADDR, m.addr);
			netv2_writel(chan->netv2_dev, CSR_ICAP_DATA_ADDR, m.data);
			netv2_writel(chan->netv2_dev, CSR_ICAP_SEND_ADDR, 1);
        }
        break;
    case NETV2_IOCTL_DMA:
        {
            struct netv2_ioctl_dma m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }
            /* fill & loopback */
            if (m.fill)
                netv2_dma_fill(chan->netv2_dev, chan->index, m.rx_tx_loopback_enable);
            netv2_writel(chan->netv2_dev, chan->dma.base + PCIE_DMA_LOOPBACK_ENABLE_OFFSET, m.tx_rx_loopback_enable);
        }
        break;
    case NETV2_IOCTL_DMA_WRITER:
        {
            struct netv2_ioctl_dma_writer m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            /* restart sync if disabled */
            if (chan->dma.writer_enable == 0) {
                netv2_writel(chan->netv2_dev, chan->dma.base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
            }
            if (m.enable) /* dma writer is disabled on release */
                netv2_writel(chan->netv2_dev, chan->dma.base + PCIE_DMA_WRITER_ENABLE_OFFSET, m.enable);
            chan->dma.writer_enable = m.enable;

            m.hw_count = chan->dma.writer_hw_count;
            m.sw_count = chan->dma.writer_sw_count;

            if (copy_to_user((void *)arg, &m, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

        }
        break;
    case NETV2_IOCTL_DMA_READER:
        {
            struct netv2_ioctl_dma_reader m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            /* restart sync / flush if disabled */
            if (chan->dma.reader_enable == 0) {
                netv2_writel(chan->netv2_dev, chan->dma.base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
            }
            if (m.enable) /* dma reader is disabled on release */
                netv2_writel(chan->netv2_dev, chan->dma.base + PCIE_DMA_READER_ENABLE_OFFSET, 1);
            chan->dma.reader_enable = m.enable;

            m.hw_count = chan->dma.reader_hw_count;
            m.sw_count = chan->dma.reader_sw_count;

            if (copy_to_user((void *)arg, &m, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

        }
        break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }
    return ret;
}

static struct file_operations netv2_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = netv2_ioctl,
    .open = netv2_open,
    .release = netv2_release,
    .read = netv2_read,
    .poll = netv2_poll,
    .write = netv2_write,
};

static int netv2_alloc_chdev(struct netv2_device *s)
{
    int i, j;
    int ret;
    int index;

    index = netv2_minor_idx;
    s->minor_base = netv2_minor_idx;
    for(i = 0; i < DMA_CHANNEL_COUNT; i++) {
        s->chan[i].cdev = cdev_alloc();
        if(!s->chan[i].cdev) {
            ret = -ENOMEM;
            printk(KERN_ERR NETV2_NAME " Failed to allocate cdev\n");
            goto fail_alloc;
        }

        cdev_init(s->chan[i].cdev, &netv2_fops);
        ret = cdev_add(s->chan[i].cdev, MKDEV(netv2_major, index), 1);
        if(ret < 0) {
            printk(KERN_ERR NETV2_NAME " Failed to allocate cdev\n");
            goto fail_alloc;
        }
        index++;
    }

    index = netv2_minor_idx;
    for(i = 0; i < DMA_CHANNEL_COUNT; i++) {
        printk(KERN_INFO NETV2_NAME " Creating /dev/netv2%d\n", index);
        if(!device_create(netv2_class, NULL, MKDEV(netv2_major, index), NULL, "netv2%d", index)) {
            ret = -EINVAL;
            printk(KERN_ERR NETV2_NAME " Failed to create device\n");
            goto fail_create;
        }
        index++;

    }

    netv2_minor_idx = index;
    return 0;

fail_create:
    index = netv2_minor_idx;
    for(j = 0; j < i ; j++){
        device_destroy(netv2_class, MKDEV(netv2_major, index++));
    }

fail_alloc:
    for(i = 0; i < DMA_CHANNEL_COUNT;i++) {
        if(s->chan[i].cdev) {
            cdev_del(s->chan[i].cdev);
            s->chan[i].cdev=NULL;
        }
    }

    return ret;
}

static void netv2_free_chdev(struct netv2_device *s)
{
    int i;

    for(i = 0; i < DMA_CHANNEL_COUNT; i++){
        device_destroy(netv2_class, MKDEV(netv2_major, s->minor_base + i));
        if(s->chan[i].cdev) {
            cdev_del(s->chan[i].cdev);
            s->chan[i].cdev=NULL;
        }
    }
}

static int netv2_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    int ret=0;
    uint8_t rev_id;
    int i, j;
    char fpga_identifier[256];

    struct netv2_device *netv2_dev = NULL;

    printk(KERN_INFO NETV2_NAME " \e[1m[Probing device]\e[0m\n");

    netv2_dev = kzalloc(sizeof(struct netv2_device), GFP_KERNEL);
    if(!netv2_dev) {
        printk(KERN_ERR NETV2_NAME " Cannot allocate memory\n");
        ret = -ENOMEM;
        goto fail1;
    }

    pci_set_drvdata(dev, netv2_dev);
    netv2_dev->dev = dev;
    spin_lock_init(&netv2_dev->reg_lock);
    list_add_tail(&(netv2_dev->list), &(netv2_list));

    ret = pci_enable_device(dev);
    if (ret != 0) {
        printk(KERN_ERR NETV2_NAME " Cannot enable device\n");
        goto fail1;
    }

    /* check device version */
    pci_read_config_byte(dev, PCI_REVISION_ID, &rev_id);
    if (rev_id != 1) {
        printk(KERN_ERR NETV2_NAME " Unsupported device version %d\n", rev_id);
        goto fail2;
    }

    if (pci_request_regions(dev, NETV2_NAME) < 0) {
        printk(KERN_ERR NETV2_NAME " Could not request regions\n");
        goto fail2;
    }


    /* check bar0 config */
    if (!(pci_resource_flags(dev, 0) & IORESOURCE_MEM)) {
        printk(KERN_ERR NETV2_NAME " Invalid BAR0 configuration\n");
        goto fail3;
    }

    netv2_dev->bar0_addr = pci_ioremap_bar(dev, 0);
    netv2_dev->bar0_size = pci_resource_len(dev, 0);
    netv2_dev->bar0_phys_addr = pci_resource_start(dev, 0);
    if (!netv2_dev->bar0_addr) {
        printk(KERN_ERR NETV2_NAME " Could not map BAR0\n");
        goto fail3;
    }

    pci_set_master(dev);
    ret = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
    if (ret) {
        printk(KERN_ERR NETV2_NAME " Failed to set DMA mask\n");
        goto fail4;
    };

    ret = pci_enable_msi(dev);
    if (ret) {
        printk(KERN_ERR NETV2_NAME " Failed to enable MSI\n");
        goto fail4;
    }

    if (request_irq(dev->irq, netv2_interrupt, IRQF_SHARED, NETV2_NAME, netv2_dev) < 0) {
        printk(KERN_ERR NETV2_NAME " Failed to allocate IRQ %d\n", dev->irq);
        goto fail5;
    }

    /* create all chardev in /dev */
    ret = netv2_alloc_chdev(netv2_dev);
    if(ret){
        printk(KERN_ERR NETV2_NAME "Failed to allocate character device\n");
        goto fail5;
    }

    for(i = 0; i < DMA_CHANNEL_COUNT; i++) {
        netv2_dev->chan[i].index = i;
        netv2_dev->chan[i].block_size = DMA_BUFFER_SIZE;
        netv2_dev->chan[i].minor = netv2_dev->minor_base + i;
        netv2_dev->chan[i].netv2_dev = netv2_dev;
        init_waitqueue_head(&netv2_dev->chan[i].wait_rd);
        init_waitqueue_head(&netv2_dev->chan[i].wait_wr);
        switch(i) {
#ifdef CSR_PCIE_DMA1_BASE
            case 1: {
                netv2_dev->chan[i].dma.base = CSR_PCIE_DMA1_BASE;
                netv2_dev->chan[i].dma.writer_interrupt = PCIE_DMA1_WRITER_INTERRUPT;
                netv2_dev->chan[i].dma.reader_interrupt = PCIE_DMA1_READER_INTERRUPT;

            }
            break;
#endif
            default:
            {
                netv2_dev->chan[i].dma.base = CSR_PCIE_DMA0_BASE;
                netv2_dev->chan[i].dma.writer_interrupt = PCIE_DMA0_WRITER_INTERRUPT;
                netv2_dev->chan[i].dma.reader_interrupt = PCIE_DMA0_READER_INTERRUPT;
            }
            break;
        }
    }

    /* allocate all dma buffers */
    ret = netv2_dma_init(netv2_dev);
    if(ret){
        printk(KERN_ERR NETV2_NAME "Failed to allocate DMA\n");
        goto fail6;
    }

    /* show identifier */
    for(i=0; i<2; i++) { /* dummy read requested at startup */
        for(j=0; j < 256; j++)
            fpga_identifier[j] = readl(netv2_dev->bar0_addr + CSR_IDENTIFIER_MEM_BASE + j*4);
    }

    printk(KERN_INFO NETV2_NAME " Version %s\n", fpga_identifier);
    printk(KERN_INFO NETV2_NAME " NeTV2 device %02x:%02x.%d assigned to minor %d to %d\n",
           dev->bus->number, PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
           netv2_dev->minor_base, netv2_dev->minor_base + DMA_CHANNEL_COUNT - 1);

    return 0;

fail6:
    netv2_free_chdev(netv2_dev);
fail5:
    pci_disable_msi(dev);
fail4:
    pci_iounmap(dev, netv2_dev->bar0_addr);
fail3:
    pci_release_regions(dev);
fail2:
    pci_disable_device(dev);
fail1:
    if(netv2_dev){
        list_del(&netv2_dev->list);
        kfree(netv2_dev);
    }
    return ret;
}

static void netv2_pci_remove(struct pci_dev *dev)
{
    struct netv2_device *netv2_dev;

    netv2_dev = pci_get_drvdata(dev);

    printk(KERN_INFO NETV2_NAME " \e[1m[Removing device]\e[0m\n");

    if(netv2_dev){
        netv2_stop_dma(netv2_dev);
        free_irq(dev->irq, netv2_dev);
        netv2_free_chdev(netv2_dev);
    }

    pci_disable_msi(dev);
    if(netv2_dev)
        pci_iounmap(dev, netv2_dev->bar0_addr);
    pci_disable_device(dev);
    pci_release_regions(dev);
    if(netv2_dev){
        netv2_dma_free(netv2_dev);
        kfree(netv2_dev);
    }
}

static const struct pci_device_id netv2_pci_ids[] = {
  { PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID ), },
  { 0, }
};
MODULE_DEVICE_TABLE(pci, netv2_pci_ids);

static struct pci_driver netv2_pci_driver = {
  .name = NETV2_NAME,
  .id_table = netv2_pci_ids,
  .probe = netv2_pci_probe,
  .remove = netv2_pci_remove,
};


static int __init netv2_module_init(void)
{
    int ret;

    netv2_class = class_create(THIS_MODULE, NETV2_NAME);
    if(!netv2_class) {
        ret = -EEXIST;
        printk(KERN_ERR NETV2_NAME " Failed to create class\n");
        goto fail_create_class;
    }

    ret = alloc_chrdev_region(&netv2_dev_t, 0, NETV2_MINOR_COUNT, NETV2_NAME);
    if(ret < 0) {
        printk(KERN_ERR NETV2_NAME " Could not allocate char device\n");
        goto fail_alloc_chrdev_region;
    }
    netv2_major = MAJOR(netv2_dev_t);
    netv2_minor_idx = MINOR(netv2_dev_t);

    ret = pci_register_driver(&netv2_pci_driver);
    if (ret < 0) {
        printk(KERN_ERR NETV2_NAME NETV2_NAME " Error while registering PCI driver\n");
        goto fail_register;
    }

    return 0;

fail_register:
    unregister_chrdev_region(netv2_dev_t, NETV2_MINOR_COUNT);
fail_alloc_chrdev_region:
    class_destroy(netv2_class);
fail_create_class:
    return ret;
}

static void __exit netv2_module_exit(void)
{
    pci_unregister_driver(&netv2_pci_driver);
    unregister_chrdev_region(netv2_dev_t, NETV2_MINOR_COUNT);
    class_destroy(netv2_class);
}


module_init(netv2_module_init);
module_exit(netv2_module_exit);

MODULE_LICENSE("GPL");
