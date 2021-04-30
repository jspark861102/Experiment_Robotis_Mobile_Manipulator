/*
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;


#include "MemoryFree.h"


int freeMemory() {
  int free_memory;

  if((int)__brkval == 0)
     free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}
*/



extern unsigned int __bss_end__;
extern unsigned int __heap_start;
extern void *__brkval;
//extern char end asm("end"); // = __bss_end__


#include "MemoryFree.h"


int freeMemory() {
  int free_memory;  
  //free_memory = ((int)&free_memory) - ((int)&__bss_end__);
  free_memory = ((int)&free_memory) - ((int)&__bss_end__) - (int)0x8000; //0x8000 - 32768
  return free_memory;
}




// from opencr_flash.ld


/* Highest address of the user mode stack */
//_estack = 0x2004FFFF;    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
//_Min_Heap_Size  = 0x8000; /* 8K required amount of heap  */
//_Min_Stack_Size = 0x2800; /* required amount of stack */

/* Specify the memory areas */
//MEMORY
//{
//FLASH (rx)      : ORIGIN = 0x08040000, LENGTH = 768K
//RAM_DTCM (xrw)  : ORIGIN = 0x20000000, LENGTH = 0x10000
//RAM (xrw)       : ORIGIN = 0x200114EC, LENGTH = 0x3EB14
//QSPI (rx)       : ORIGIN = 0x90000000, LENGTH = 16M
//}
