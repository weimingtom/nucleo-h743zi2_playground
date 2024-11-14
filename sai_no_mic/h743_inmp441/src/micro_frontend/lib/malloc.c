#include "malloc.h"	   
//#include "ucos_ii.h"	 
//see cmsis_armcc.h
#ifndef __ALIGNED
#define __ALIGNED(x) __attribute__((aligned(x)))
#endif
//__align(x) is for AC5 compiler, so change it
#define __align(x) __ALIGNED(x)

__align(MEM1_BLOCK_SIZE) uint8_t mem1base[MEM1_MAX_SIZE];
#if SRAMBANK>=2
__align(MEM2_BLOCK_SIZE) uint8_t mem2base[MEM2_MAX_SIZE] __attribute__((at(0XC01F4000)));
#endif
#if SRAMBANK>=3
__align(MEM3_BLOCK_SIZE) uint8_t mem3base[MEM3_MAX_SIZE] __attribute__((at(0X10000000)));
#endif
uint32_t mem1mapbase[MEM1_ALLOC_TABLE_SIZE];
#if SRAMBANK>=2
uint32_t mem2mapbase[MEM2_ALLOC_TABLE_SIZE] __attribute__((at(0XC01F4000+MEM2_MAX_SIZE)));
#endif
#if SRAMBANK>=3
uint32_t mem3mapbase[MEM3_ALLOC_TABLE_SIZE] __attribute__((at(0X10000000+MEM3_MAX_SIZE)));
#endif

const uint32_t memtblsize[SRAMBANK]={MEM1_ALLOC_TABLE_SIZE,
#if SRAMBANK>=2
MEM2_ALLOC_TABLE_SIZE,
#endif
#if SRAMBANK>=3
MEM3_ALLOC_TABLE_SIZE,
#endif
};
const uint32_t memblksize[SRAMBANK]={MEM1_BLOCK_SIZE,
#if SRAMBANK>=2
MEM2_BLOCK_SIZE,
#endif
#if SRAMBANK>=3
MEM3_BLOCK_SIZE,
#endif
};
const uint32_t memsize[SRAMBANK]={MEM1_MAX_SIZE,
#if SRAMBANK>=2
MEM2_MAX_SIZE,
#endif
#if SRAMBANK>=3
MEM3_MAX_SIZE,
#endif
};

struct _m_mallco_dev mallco_dev=
{
	my_mem_init,
	my_mem_perused,

//see SRAMBANK==3, three bank!!!
	mem1base,
#if SRAMBANK>=2
mem2base,
#endif
#if SRAMBANK>=3
mem3base,
#endif
	mem1mapbase,
#if SRAMBANK>=2
mem2mapbase,
#endif
#if SRAMBANK>=3
mem3mapbase,
#endif
	0,
#if SRAMBANK>=2
0,
#endif
#if SRAMBANK>=3
0,
#endif
};


void mymemcpy(void *des,void *src,uint32_t n)  
{  
    uint8_t *xdes=des;
    uint8_t *xsrc=src; 
    while(n--)*xdes++=*xsrc++;  
}  

void mymemset(void *s,uint8_t c,uint32_t count)  
{  
    uint8_t *xs = s;  
    while(count--)*xs++=c;  
}	

void my_mem_init(uint8_t memx)  
{  
    mymemset(mallco_dev.memmap[memx],0,memtblsize[memx]*4);
 	mallco_dev.memrdy[memx]=1;
}  

uint16_t my_mem_perused(uint8_t memx)  
{  
    uint32_t used=0;  
    uint32_t i;  
    for(i=0;i<memtblsize[memx];i++)  
    {  
        if(mallco_dev.memmap[memx][i])used++; 
    } 
    return (used*1000)/(memtblsize[memx]);  
}  

uint32_t my_mem_malloc(uint8_t memx,uint32_t size)  
{  
    signed long offset=0;  
    uint32_t nmemb;
    uint32_t cmemb=0;
    uint32_t i;  
    if(!mallco_dev.memrdy[memx])mallco_dev.init(memx);
    if(size==0)return 0XFFFFFFFF;
    nmemb=size/memblksize[memx]; 
    if(size%memblksize[memx])nmemb++;  
    for(offset=memtblsize[memx]-1;offset>=0;offset--)
    {     
		if(!mallco_dev.memmap[memx][offset])cmemb++;
		else cmemb=0;
		if(cmemb==nmemb)
		{
            for(i=0;i<nmemb;i++) 
            {  
                mallco_dev.memmap[memx][offset+i]=nmemb;  
            }  
            return (offset*memblksize[memx]);
		}
    }  
    return 0XFFFFFFFF;
}  

uint8_t my_mem_free(uint8_t memx,uint32_t offset)  
{  
    int i;  
    if(!mallco_dev.memrdy[memx])
	{
		mallco_dev.init(memx);    
        return 1;
    }  
    if(offset<memsize[memx])
    {  
        int index=offset/memblksize[memx];
        int nmemb=mallco_dev.memmap[memx][index];
        for(i=0;i<nmemb;i++) 
        {  
            mallco_dev.memmap[memx][index+i]=0;  
        }  
        return 0;  
    }else return 2;
}  

void myfree(uint8_t memx,void *ptr)  
{  
//    OS_CPU_SR cpu_sr=0;
	uint32_t offset;   
	if(ptr==NULL)return;		  
//	OS_ENTER_CRITICAL();		
 	offset=(uint32_t)ptr-(uint32_t)mallco_dev.membase[memx];     
    my_mem_free(memx,offset);	
//	OS_EXIT_CRITICAL();    
}  

void *mymalloc(uint8_t memx,uint32_t size)  
{  
//    OS_CPU_SR cpu_sr=0;
    uint32_t offset;   
//	OS_ENTER_CRITICAL();			
	offset=my_mem_malloc(memx,size);  
//	OS_EXIT_CRITICAL();        	 	   
    if(offset==0XFFFFFFFF)return NULL;  
    else return (void*)((uint32_t)mallco_dev.membase[memx]+offset);  
}  

void *myrealloc(uint8_t memx,void *ptr,uint32_t size)  
{  
    uint32_t offset;    
    offset=my_mem_malloc(memx,size);   	
    if(offset==0XFFFFFFFF)return NULL;     
    else  
    {  									   
	    mymemcpy((void*)((uint32_t)mallco_dev.membase[memx]+offset),ptr,size);	  
        myfree(memx,ptr);  
        return (void*)((uint32_t)mallco_dev.membase[memx]+offset);  				
    }  
}












