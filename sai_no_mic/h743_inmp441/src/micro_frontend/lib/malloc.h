#ifndef __MALLOC_H
#define __MALLOC_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "sys.h" 
#include <stdint.h>
//typedef uint32_t u32;
//typedef uint16_t u16;
//typedef uint8_t u8; 

#ifndef NULL
#define NULL 0
#endif

#define SRAMIN	 0
#define SRAMEX   1
#define SRAMCCM  2

//#define SRAMBANK 	3
//I only need 1 bank
#define SRAMBANK 	1

#define MEM1_BLOCK_SIZE			64
//FIXME:for micro_speech micro_frontend, at least 14k (if 13K, mymalloc will return null)
#define MEM1_MAX_SIZE			20*1024//14*1024//150*1024 
#define MEM1_ALLOC_TABLE_SIZE	MEM1_MAX_SIZE/MEM1_BLOCK_SIZE

#define MEM2_BLOCK_SIZE			64
#define MEM2_MAX_SIZE			28912 *1024
#define MEM2_ALLOC_TABLE_SIZE	MEM2_MAX_SIZE/MEM2_BLOCK_SIZE

#define MEM3_BLOCK_SIZE			64 
#define MEM3_MAX_SIZE			60 *1024 
#define MEM3_ALLOC_TABLE_SIZE	MEM3_MAX_SIZE/MEM3_BLOCK_SIZE 

struct _m_mallco_dev
{
	void (*init)(uint8_t);
	uint16_t (*perused)(uint8_t);
	uint8_t *membase[SRAMBANK];
	uint32_t *memmap[SRAMBANK];
	uint8_t  memrdy[SRAMBANK];
};
extern struct _m_mallco_dev mallco_dev;

void mymemset(void *s,uint8_t c,uint32_t count);
void mymemcpy(void *des,void *src,uint32_t n);   
void my_mem_init(uint8_t memx);
uint32_t my_mem_malloc(uint8_t memx,uint32_t size);
uint8_t my_mem_free(uint8_t memx,uint32_t offset);
uint16_t my_mem_perused(uint8_t memx) ;
////////////////////////////////////////////////////////////////////////////////

void myfree(uint8_t memx,void *ptr); 
void *mymalloc(uint8_t memx,uint32_t size);
void *myrealloc(uint8_t memx,void *ptr,uint32_t size);


#ifdef __cplusplus
}
#endif

#endif













