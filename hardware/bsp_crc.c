#include "bsp_crc.h"

uint32_t bsp_crc_calc(uint32_t *data, uint32_t len)
{
	CRC_ResetDR();

	return CRC_CalcBlockCRC(data, len);
}

void bsp_crc_init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}
