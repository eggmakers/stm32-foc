#include "FlashStorage.h"
#include <string.h>

#define FLASH_STORAGE_PARAM_ADDR 0x08007800 // ����ṹ��������Flash��30ҳ
#define FLASH_STORAGE_ID_ADDR 0x08007C00	// ���ID����Flash��31ҳ
#define FLASH_STORAGE_AVAIL_FLAG 0xA5A5A5A5 // Flash������Ч��־

// ��������ṹ����
void Flash_EraseMotorParam()
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = FLASH_STORAGE_PARAM_ADDR;
	erase.NbPages = 1;
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase, &error);
	HAL_FLASH_Lock();
}

// ������ṹ��������Flash
void Flash_SaveMotorParam(int poles, float zero_elec_angle, int dir)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = FLASH_STORAGE_PARAM_ADDR;
	erase.NbPages = 1;
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase, &error);

	uint32_t buf[5];
	buf[0] = FLASH_STORAGE_AVAIL_FLAG;
	buf[1] = *((uint32_t *)&poles);
	buf[2] = *((uint32_t *)&zero_elec_angle);
	buf[3] = *((uint32_t *)&dir);
	for (uint8_t i = 0; i < sizeof(buf); i++)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_PARAM_ADDR + i * 4, buf[i]);

	HAL_FLASH_Lock();
}

// ��Flash�ж�ȡ����ṹ����
int Flash_ReadMotorParam(int *poles, float *zero_elec_angle, int *dir)
{
	uint32_t *buf = (uint32_t *)FLASH_STORAGE_PARAM_ADDR;
	if (buf[0] != FLASH_STORAGE_AVAIL_FLAG)
		return -1;
	*poles = *((int *)&buf[1]);
	*zero_elec_angle = *((float *)&buf[2]);
	*dir = *((int *)&buf[3]);
	return 0;
}

// �����ID����Flash
void Flash_SaveMotorID(uint8_t id)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = FLASH_STORAGE_ID_ADDR;
	erase.NbPages = 1;
	uint32_t error = 0;
	HAL_FLASHEx_Erase(&erase, &error);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ID_ADDR, FLASH_STORAGE_AVAIL_FLAG);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ID_ADDR + 4, id);

	HAL_FLASH_Lock();
}

// ��Flash�ж�ȡ���ID
uint8_t Flash_ReadMotorID()
{
	uint32_t *buf = (uint32_t *)FLASH_STORAGE_ID_ADDR;
	if (buf[0] != FLASH_STORAGE_AVAIL_FLAG)
		return 0;
	return buf[1];
}
