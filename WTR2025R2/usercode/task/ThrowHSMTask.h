/*
 * @Description: 抛射全车状态机更改线程
 * @Author: Alex
 * @Date: 2025-03-10 16:06:28
 * @LastEditors: Alex
 * @LastEditTime: 2025-03-23 22:16:18
 */

#ifndef THROWHSMTASK_H
#define THROWHSMTASK_H

#include "AllUserTask.h"
#include "main.h"
#include "rmctl.h"
#include "ThrowHSM.h"
#include "encoder.h"
#include "DJI.h"
#include "BounceTask.h"
#include "CommunicationTask.h"

void ThrowHSMTask(void* argument);

extern int TimeUp;

#endif