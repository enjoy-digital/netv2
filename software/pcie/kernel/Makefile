# Makefile for kernel module

KERNEL_VERSION:=$(shell uname -r)
KERNEL_PATH:=/lib/modules/$(KERNEL_VERSION)/build

obj-m = netv2.o
netv2-objs = main.o

all: netv2.ko

netv2.ko: main.c
	make -C $(KERNEL_PATH) M=$(shell pwd) modules

clean:
	make -C $(KERNEL_PATH) M=$(shell pwd) clean
	rm -f *~
