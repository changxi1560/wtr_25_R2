/*
 * @Description: 抛射执行线程
 * @Author: Alex
 * @Date: 2025-03-02 19:35:06
 * @LastEditors: Alex
 * @LastEditTime: 2025-04-17 00:40:05
 */

#include "ThrowTask.h"



/**
 * @brief 抛射执行线程
 * 
 * @param argument 
 */
void ThrowTask(void *argument)
{
    uint8_t flag_countBraceDelay = 0; //标志变量 用于支撑结构打开一段时间后置电流为0
    uint8_t flag_countPawlDelay = 0;  //标志变量 用于夹爪关闭一段时间后置电流为0
    float torque = 0;
    for(;;)   
    {
        if (throwhsm.wholestate == WHOLE_THROW || throwhsm.wholestate == WHOLE_CONTHROW)
        {
            if(throwhsm.throwstate == THROW_IDLE)
            {
                for (uint8_t i = 2; i < 4; ++i) hDJI[i].speedPID.output = 0;
                hDJI[0].speedPID.output = 0;
                hDJI[1].speedPID.output = -6000;
                Unitree_ChangeState(&myMotor0, 0, 1, 0, 0, 0, 0, 0);
                Unitree_ChangeState(&myMotor1, 1, 1, 0, 0, 0, 0, 0);
                if(rmctl.rmctl_msg.btn_RightCrossRight == 1)
                {
                    torque = 3.2;
                }
                else if(rmctl.rmctl_msg.btn_RightCrossUp == 1)
                {
                    torque = 2.8;
                }
                else if(rmctl.rmctl_msg.btn_RightCrossLeft == 1)
                {
                    torque = 2.4;
                }
                else if(rmctl.rmctl_msg.btn_RightCrossDown == 1)
                {
                    torque = 2.0;
                }
            }
            else if (throwhsm.throwstate == THROW_CATCHING)
            {
                hDJI[1].speedPID.output = -6000;
            }
            else if (throwhsm.throwstate == THROW_GATHERSTRENGTH)
            {
                hDJI[0].speedPID.output = 0;
                hDJI[1].speedPID.output = -6000;
                Unitree_ChangeState(&myMotor0, 0, 1, 0, 0, 0, 0, 0);
                Unitree_ChangeState(&myMotor1, 1, 1, 2.8, 0.15, unitreeStartPos1+1.9, 0.55, 0.15);
                hDJI[2].speedPID.output = 0;
                hDJI[3].speedPID.output = 0;
            }
            else if (throwhsm.throwstate == THROW_CLEARBRACE)
            {
                while (flag_countBraceDelay < 50) 
                {
                    speedServo(5000, &hDJI[0]);
                    hDJI[1].speedPID.output = -6000;
                    hDJI[2].speedPID.output = -5000;
                    hDJI[3].speedPID.output = 5000;
                    ++flag_countBraceDelay;
                    osDelay(2);
                }
                speedServo(5000, &hDJI[0]);
                hDJI[1].speedPID.output = -6000;
                hDJI[2].speedPID.output = 0;
                hDJI[3].speedPID.output = 0;
            }
            else if (throwhsm.throwstate == THROW_ACCELERATE)
            {
                speedServo(5000, &hDJI[0]); 
                hDJI[1].speedPID.output = -6000;

                //纯力矩方式测算
                Unitree_Update(torque);
            }
            else if (throwhsm.throwstate == THROW_THROWOUT)
            {
                hDJI[1].speedPID.output = 6000;
                //纯力矩方式测算
                Unitree_Update(torque);
            }
            else if (throwhsm.throwstate == THROW_BACK)
            {
                hDJI[0].speedPID.output = 0;

                while (flag_countPawlDelay < 200)
                {
                    Unitree_ChangeState(&myMotor0, 0, 1, 0, -0.8, 0, 0, 0.3);
                    Unitree_ChangeState(&myMotor1, 1, 1, 0,  0 , 0, 0, 0);
                    hDJI[1].speedPID.output = -6000;
                    ++flag_countPawlDelay;
                    osDelay(2);
                }
                hDJI[1].speedPID.output = 0;
                Unitree_ChangeState(&myMotor0, 0, 1, 0, -0.01, 0, 0, 0.3);
                Unitree_ChangeState(&myMotor1, 1, 1, 0,  0 , 0, 0, 0);
            }
            osDelay(2);
        }
        else
        {
            flag_countBraceDelay = 0;
            flag_countPawlDelay = 0;
            osDelay(10);
        }
    }


  /* USER CODE END encoderRead */
}


void Unitree_Update(float torque)
{
    float k = 1.02;
    Unitree_ChangeState(&myMotor0, 0, 1,  torque - k * sin((180 - data.angle )/90.0 *M_PI_2), 0, 0, 0, 0);
    Unitree_ChangeState(&myMotor1, 1, 1, -torque + k * sin((180 - data.angle )/90.0 *M_PI_2), 0, 0, 0, 0);
}