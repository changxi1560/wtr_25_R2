/*
 * @Description: 拍球执行线程
 * @Author: Alex
 * @Date: 2025-03-02 19:35:06
 * @LastEditors: Alex
 * @LastEditTime: 2025-03-28 18:11:14
 */

#include "BounceTask.h"

uint8_t externFlag_startSensor = 0;  //信号量 用于指示状态机在何时开始拍球返回检测
uint8_t timeflag =0;
uint8_t deltime =0;//延时变量
uint8_t flag_countPawlBack = 0; //标志变量 用于夹爪快速抓球一段时间后转位置伺服
uint32_t flag_countBounceCylinderContract = 0;  //标志变量 用于拍球气缸向下拍一段时间后收回
uint8_t timer_active_flag = 0; //定时器激活标志符
void BounceTask(void* argument)
{
    uint8_t flag_countUnitreeRelease = 0;  //标志变量 用于支撑结构关闭一段时间后置电流为零，宇树卸力
    for(;;)
    {
        if (throwhsm.wholestate == WHOLE_BOUNCE || throwhsm.wholestate == WHOLE_CONBOUNCE)
        // if(1)
        {
            if (throwhsm.bouncestate == BOUNCE_CATCHING)
            {
                hDJI[1].speedPID.output = -6000;//爪子关
                speedServo(5000, &hDJI[0]);
            }
            else if (throwhsm.bouncestate == BOUNCE_GETTOPOSITION)
            {
                hDJI[1].speedPID.output = -6000;//爪子关
                Unitree_ChangeState(&myMotor0, 0, 1, 0, 0, 0, 0, 0);
                Unitree_ChangeState(&myMotor1, 1, 1, 2.8, 0.15, unitreeStartPos1+1.9, 0.55, 0.15);
                hDJI[0].speedPID.output = 0;
            }
            else if (throwhsm.bouncestate == BOUNCE_READY)
            {
                while (flag_countUnitreeRelease < 250)
                {
                    hDJI[1].speedPID.output = -6000;//爪子关
                    hDJI[2].speedPID.output = 8000;
                    hDJI[3].speedPID.output = -8000;
                    ++flag_countUnitreeRelease;
                    osDelay(2);
                }
                hDJI[1].speedPID.output = -6000;//爪子关
                hDJI[2].speedPID.output = 0;
                hDJI[3].speedPID.output = 0;
                positionServo(-115, &hDJI[1]);
                Unitree_ChangeState(&myMotor0, 0, 1, 0, 0, 0, 0, 0);
                Unitree_ChangeState(&myMotor1, 1, 1, 0, 0, 0, 0, 0);
                HAL_GPIO_WritePin(CYLIN_GPIO_Port,CYLIN_Pin,GPIO_PIN_SET);  //伸缩气缸推出
            }
            else if (throwhsm.bouncestate == BOUNCE_BOUNCE)
            {
                while (flag_countBounceCylinderContract < 100)
                {
                    hDJI[1].speedPID.output = 6000;//爪子开
                    HAL_GPIO_WritePin(BOUNCE_GPIO_Port,BOUNCE_Pin,GPIO_PIN_SET);  //拍球气缸推出
                    ++flag_countBounceCylinderContract;
                    osDelay(2);
                }
                HAL_GPIO_WritePin(BOUNCE_GPIO_Port,BOUNCE_Pin,GPIO_PIN_RESET);  //拍球气缸收回
                while(timeflag<75)
                {
                    timeflag++;
                    hDJI[1].speedPID.output = 0;
                }

                externFlag_startSensor = 1;

                if(timer_active_flag == 0)
                {
                // 启动前确保状态干净
                __HAL_TIM_SET_COUNTER(&htim3, 0);
                __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
                
                // 先清除可能存在的旧中断标志
                if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
                    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
                }
                
                timer_active_flag = 1;
                HAL_TIM_Base_Start_IT(&htim3); // 最后启动
                }
            }
            else if (throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE)
            {
                // 增加三重保护确保定时器完全停止
                speedServo(0,&hDJI[0]);
                if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
                    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
                }
                if (htim3.Instance->CR1 & TIM_CR1_CEN) {
                    HAL_TIM_Base_Stop_IT(&htim3);
                }
                __HAL_TIM_SET_COUNTER(&htim3, 0);
                
                timer_active_flag = 0; // 增加标志位复位 🔴 关键修复
                while (flag_countPawlBack < 50)
                {
                    hDJI[1].speedPID.output = -6000;
                    ++flag_countPawlBack;
                    osDelay(2);
                }
                hDJI[1].speedPID.output = -6000;
                flag_countBounceCylinderContract = 0;
                externFlag_startSensor = 0;
            }
            else if (throwhsm.bouncestate == BOUNCE_WAITBALL)
            {
                hDJI[1].speedPID.output = 6000;//开
                hDJI[0].speedPID.output = 0;
            }
            else if (throwhsm.bouncestate == BOUNCE_ADJUSTPOSTURE)
            {
                hDJI[1].speedPID.output = -6000;//爪子关
                speedServo(-4000, &hDJI[0]);
            }
            else if (throwhsm.bouncestate == BOUNCE_READYTHROW)
            {
                hDJI[1].speedPID.output = -6000;//爪子关
                speedServo(-5000, &hDJI[0]);//自旋
                HAL_GPIO_WritePin(CYLIN_GPIO_Port,CYLIN_Pin,GPIO_PIN_RESET); //伸缩气缸收回
            }
            osDelay(2);
        }
        else
        {
            flag_countBounceCylinderContract = 0;
            flag_countUnitreeRelease = 0;
            flag_countPawlBack = 0;
            externFlag_startSensor = 0;
            deltime = 0;
            osDelay(10);
        }
    }
}