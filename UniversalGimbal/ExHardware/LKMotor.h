#ifndef __LK_MOTOR_H_
#define __LK_MOTOR_H_

/**
 * @attention   采用UTF-8字符集编码
 */

#include "header.h"

typedef Motor_t LKMotor_t;
typedef LKMotor_t MF7015_t;

void MF7015_Init(LKMotor_t *motor, int ID);
void LKMotor_TryUpdate(int ID, uint8_t data[8]);
void LKMotor_Update(LKMotor_t *motor, uint8_t data[8]);
void LKMotor_Output(LKMotor_t *motor);
void LKMotor_Enable(LKMotor_t *motor);
void LKMotor_Disable(LKMotor_t *motor);

#endif 
