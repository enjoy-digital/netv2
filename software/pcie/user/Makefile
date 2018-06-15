CFLAGS=-O2 -Wall -g -I../kernel -MMD
LDFLAGS=-g
CC=gcc
AR=ar

PROGS=netv2_util

all: $(PROGS)

netv2_util: libnetv2.o netv2_util.o
	$(CC) $(LDFLAGS) -o $@ $^ -lrt -lm

clean:
	rm -f $(PROGS) *.o *.a *.d *~

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

-include $(wildcard *.d)
