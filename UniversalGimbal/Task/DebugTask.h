#ifndef _DEBUG_TASK_H_
#define _DEBUG_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"

typedef struct 
{
    uint32_t Head;
    float Value[6];
    uint32_t Tail;

    float TempValue[4];
} DebugData_t;

void DebugTask(void const * argument);

extern volatile DebugData_t DebugData;

#endif
