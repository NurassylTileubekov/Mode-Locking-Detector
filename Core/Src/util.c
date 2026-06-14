#include "../Inc/util.h"
#include <stdint.h>
#include "stm32g4xx_hal.h"

uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

uint32_t GetBank(uint32_t Addr)
{
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    return FLASH_BANK_1;
  }
  else
  {
    return FLASH_BANK_2;
  }
}

uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{
    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError;

    HAL_FLASH_Unlock();

    uint32_t StartPage = GetPage(StartPageAddress);
    uint32_t EndPageAdress = StartPageAddress + numberofwords * 4;
    uint32_t EndPage = GetPage(EndPageAdress);

    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = GetBank(StartPageAddress);
    EraseInitStruct.Page        = StartPage;
    EraseInitStruct.NbPages     = (EndPage - StartPage) + 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_FLASH_GetError();
    }

    for (int i = 0; i < numberofwords; i += 2)
    {
        uint64_t data64;
        if (i + 1 < numberofwords)
        {
            data64 = ((uint64_t)Data[i+1] << 32) | (uint64_t)Data[i];
        }
        else
        {
            data64 = 0xFFFFFFFF00000000 | (uint64_t)Data[i];
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, data64) == HAL_OK)
        {
            StartPageAddress += 8;
        }
        else
        {
            HAL_FLASH_Lock();
            return HAL_FLASH_GetError();
        }
    }

    HAL_FLASH_Lock();
    return 0;
}

void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
    for (int i = 0; i < numberofwords; i++)
    {
        RxBuf[i] = *(__IO uint32_t *)StartPageAddress;
        StartPageAddress += 4;
    }
}
uint8_t struct_cmp(void* struct_1, void* struct_2, int size) {
    uint8_t* ptr_1 = (uint8_t*)struct_1;
    uint8_t* ptr_2 = (uint8_t*)struct_2;
    for (int i = 0; i < size; i++) {
        if (ptr_1[i] != ptr_2[i]) {
            return 0;
        }
    }
    return 1;
}
void struct_copy(void* struct_1, void* struct_2, int size) {
    uint8_t* ptr_1 = (uint8_t*)struct_1;
    uint8_t* ptr_2 = (uint8_t*)struct_2;
    for (int i = 0; i < size; i++) {
        ptr_2[i] = ptr_1[i];
    }
}
