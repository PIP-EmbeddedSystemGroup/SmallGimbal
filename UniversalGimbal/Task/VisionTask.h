#ifndef _VISION_TASK_H_
#define _VISION_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"
#include "header.h"
#include "UniversalPID.h"

/* 协议 v5.2 起，自瞄工作模式 / 打击决策优先级改为位域取值（原 v5.1 的
 * 0xB4/0xF3/0xA8/0xC0/0xE6 枚举已废弃），此处的枚举值即协议 3.2 字节 12 的位域编码 */
typedef enum
{
    VISION_MODE_OFF = 0,
    VISION_MODE_ARMOR = 1,
    VISION_MODE_OUTPOST = 2,
    VISION_MODE_POWER_RUNE_LITTLE = 3,
    VISION_MODE_POWER_RUNE_BIG = 4
}
VisionMode_t;

typedef enum
{
    DECISION_MODE_AOTU = 0,
    DECISION_MODE_DISTANCE = 1,
    DECISION_MODE_SCREEN = 2   
}
DecisionMode_t;

typedef struct
{
    NUC_t *NUC;

    VisionMode_t VisionMode;
    DecisionMode_t DecisionMode;
 
} Vision_t;

void VisionTask(void const *argument);

extern Vision_t Vision;

#endif
