/*
 * @Description: 拍球执行线程
 * @Author: Alex
 * @Date: 2025-03-12 15:04:54
 * @LastEditors: Alex
 * @LastEditTime: 2025-03-12 15:06:00
 */

#ifndef BOUNCETASK_H
#define BOUNCETASK_H

#include "AllUserTask.h"
#include "main.h"
#include "ThrowHSM.h"
#include "Unitree_user.h"
#include "DJI.h"
#include "tim.h"
#include "main.h"
// #include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_tim.h"  // 根据您的芯片系列（如 stm32f4xx_hal_tim.h）

extern uint8_t externFlag_startSensor;
extern uint8_t deltime;
extern uint8_t timer_active_flag;
void BounceTask(void* argument);

#endif