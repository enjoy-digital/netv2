#ifndef __DUMP_H
#define __DUMP_H

void mr(char *startaddr, char *len);
void mw(char *addr, char *value, char *count);
void mc(char *dstaddr, char *srcaddr, char *count);

#endif
