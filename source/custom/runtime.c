// SPDX-License-Identifier: GPL-3.0-or-later
#include <stddef.h>
void *memset(void *dst,int c,size_t n) {
    unsigned char *d=dst;for(size_t i=0;i<n;++i)d[i]=(unsigned char)c;return dst;
}
void *memcpy(void *dst,const void *src,size_t n) {
    unsigned char *d=dst;const unsigned char *s=src;
    for(size_t i=0;i<n;++i)d[i]=s[i];
    return dst;
}
void __aeabi_memclr(void *dst,size_t n) {memset(dst,0,n);}
void __aeabi_memclr4(void *dst,size_t n) {memset(dst,0,n);}
void __aeabi_memclr8(void *dst,size_t n) {memset(dst,0,n);}
void __aeabi_memcpy(void *dst,const void *src,size_t n) {memcpy(dst,src,n);}
void __aeabi_memcpy4(void *dst,const void *src,size_t n) {memcpy(dst,src,n);}
