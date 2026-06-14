#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);
void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);
uint32_t GetPage(uint32_t Addr);
uint32_t GetBank(uint32_t Addr);
uint8_t struct_cmp(void* struct_1, void* struct_2, int size);
void struct_copy(void* struct_1, void* struct_2, int size);
#endif
