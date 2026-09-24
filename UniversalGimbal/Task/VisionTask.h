#ifndef _VISION_TASK_H_
#define _VISION_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "stdint.h"
#include "cmsis_os.h"
#include "header.h"
#include "UniversalPID.h"

typedef enum
{
    VISION_MODE_OFF = 0xB4,
    VISION_MODE_ARMOR = 0xF3,
    VISION_MODE_OUTPOST = 0xA8,
    VISION_MODE_POWER_RUNE_LITTLE = 0xC0,
    VISION_MODE_POWER_RUNE_BIG = 0xE6
}
VisionMode_t;

typedef enum
{
    DECISION_MODE_AOTU = 0xB4,
    DECISION_MODE_DISTANCE = 0xF3,
    DECISION_MODE_SCREEN = 0xA8   
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
