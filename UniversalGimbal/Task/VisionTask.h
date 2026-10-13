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
    VISION_MODE_OFF = 0,
    VISION_MODE_ARMOR = 1,
    VISION_MODE_OUTPOST = 2,
    VISION_MODE_POWER_RUNE_LITTLE = 3,
    VISION_MODE_POWER_RUNE_BIG = 4,
    VISION_MODE_FULL_AUTO = 5       
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
