#ifndef __CRC_H
#define __CRC_H

#include "main.h"

#define CRC16_INIT 0xFFFF

uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC);
int Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

#endif
