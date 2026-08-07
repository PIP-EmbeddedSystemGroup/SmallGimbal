#ifndef _UNIVERSAL_CRC_H_
#define _UNIVERSAL_CRC_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"

// CRC16多项式定义 (常用的几种) 
#define CRC16_CCITT_POLY    0x1021  // x^16 + x^12 + x^5 + 1 
#define CRC16_IBM_POLY      0x8005  // x^16 + x^15 + x^2 + 1 
#define CRC16_MODBUS_POLY   0xA001  // x^16 + x^15 + x^2 + 1 (反向) 
 
// CRC16初始值定义 
#define CRC16_CCITT_INIT    0xFFFF 
#define CRC16_IBM_INIT      0x0000 
#define CRC16_MODBUS_INIT   0xFFFF 
 
// CRC16计算函数类型 
typedef enum { 
    CRC16_CCITT = 0, 
    CRC16_IBM, 
    CRC16_MODBUS 
} CRC16_Type_t; 


uint8_t CRC8_GetChecksum(uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t CRC8_VerifyChecksum(uint8_t* pchMessage, uint32_t dwLength);
void CRC8_AppendChecksum(uint8_t* pchMessage, uint32_t dwLength);
uint16_t CRC16_GetChecksum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t CRC16_VerifyChecksum(uint8_t* pchMessage, uint32_t dwLength);
void CRC16_AppendChecksum(uint8_t* pchMessage, uint32_t dwLength);


HAL_StatusTypeDef CRC16_Verify(const uint8_t *data, uint16_t len, CRC16_Type_t );
uint16_t CRC16_Append(uint8_t *data, uint16_t data_len, CRC16_Type_t crc_type);
#endif
