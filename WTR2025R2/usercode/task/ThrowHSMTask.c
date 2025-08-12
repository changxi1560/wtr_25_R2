/*
 * @Description: 抛射全车状态机更改线程
 * @Author: Alex
 * @Date: 2025-03-02 19:43:40
 * @LastEditors: Alex
 * @LastEditTime: 2025-04-17 00:53:39
 */

#include "ThrowHSMTask.h"
int TimeUp = 0;
/**
 * @brief 抛射全车状态机修改线程
 * 
 * @param argument 
 */
void ThrowHSMTask(void* argument)
{
    static bool risedownflag1 = 0, onceflag1= 0,risedownflag2 = 0, onceflag2= 0;
    for(;;)
    {
        if (flag_initComplete == 1)
        {
            //整车状态修改
            //当所有电机初始化工作完成后，整车就绪态
            if (flag_initComplete == 1 && throwhsm.wholestate != WHOLE_ERROR)  throwhsm.wholestate = WHOLE_READY;  
            if (throwhsm.wholestate != WHOLE_INITING && throwhsm.wholestate != WHOLE_ERROR)
            {
                //左开关打开上，准备态
                if(rmctl.rmctl_msg.left_switch == 1)
                {
                    throwhsm.wholestate = WHOLE_READY;
                    throwhsm.throwstate = THROW_IDLE;
                    throwhsm.bouncestate = BOUNCE_IDLE;
                }
                //左开关闭合下，右开关打开上，拍球模式
                else if (rmctl.rmctl_msg.left_switch == 0 && rmctl.rmctl_msg.right_switch == 1)
                {
                    if (rmctl.rmctl_msg.usr_left_knob >= 8) throwhsm.wholestate = WHOLE_CONBOUNCE; //连续拍球模式
                    else throwhsm.wholestate = WHOLE_BOUNCE;
                    throwhsm.throwstate = THROW_IDLE;
                }
                //左右开关都向下，抛球模式
                else if (rmctl.rmctl_msg.left_switch == 0 && rmctl.rmctl_msg.right_switch == 0)
                {
                    if (rmctl.rmctl_msg.usr_left_knob >= 8) throwhsm.wholestate = WHOLE_CONTHROW; //连续抛球模式
                    else throwhsm.wholestate = WHOLE_THROW;
                    throwhsm.bouncestate = BOUNCE_IDLE;
                }
            }
            //抛球状态修改
            if (throwhsm.wholestate == WHOLE_THROW)
            {
                if(rmctl.rmctl_msg.btn_Btn4 == 1)  throwhsm.throwstate = THROW_RIDARPOSITIONSERVE;
                if(rmctl.rmctl_msg.btn_Btn5 == 1)  throwhsm.throwstate = THROW_CAMERAPOSITIONSERVE;
                if (throwhsm.throwstate == THROW_RIDARPOSITIONSERVE && MyData.AnglePID.cur_error<0.1 && MyData.AnglePID.cur_error> -0.1)   throwhsm.throwstate = THROW_READY;
                if (throwhsm.throwstate == THROW_CAMERAPOSITIONSERVE && MyData.CameraPID.cur_error<0.1 && MyData.CameraPID.cur_error> -0.1)   throwhsm.throwstate = THROW_READY;

                
                if ((throwhsm.throwstate == THROW_IDLE || throwhsm.throwstate == THROW_RESET1 )&& rmctl.rmctl_msg.btn_Btn0 == 1 )  throwhsm.throwstate = THROW_GATHERSTRENGTH;
                if (throwhsm.throwstate == THROW_GATHERSTRENGTH && rmctl.rmctl_msg.btn_Btn2 == 1 )  throwhsm.throwstate = THROW_CLEARBRACE;
                if ((throwhsm.throwstate == THROW_CLEARBRACE || throwhsm.throwstate == THROW_READY) && rmctl.rmctl_msg.btn_Btn3 == 1)  throwhsm.throwstate = THROW_ACCELERATE;
                if (throwhsm.throwstate == THROW_ACCELERATE && caldata.angle>=19500.0)  throwhsm.throwstate = THROW_THROWOUT;
                if (throwhsm.throwstate == THROW_THROWOUT && (hDJI[1].Calculate.RotorRound > -1 || caldata.angle >= 28000.0))  throwhsm.throwstate = THROW_BACK;
                
            }
            //连续抛球状态修改
            else if (throwhsm.wholestate == WHOLE_CONTHROW)
            {
                if ((throwhsm.throwstate == THROW_IDLE || throwhsm.throwstate == THROW_BACK) &&rmctl.rmctl_msg.btn_Btn0 == 1) 
                {
                    throwhsm.throwstate = THROW_CATCHING;
                }
                if (throwhsm.throwstate == THROW_CATCHING && rmctl.rmctl_msg.btn_Btn1 == 1)  throwhsm.throwstate = THROW_GATHERSTRENGTH;
                if (throwhsm.throwstate == THROW_GATHERSTRENGTH && rmctl.rmctl_msg.btn_Btn2 == 1)  throwhsm.throwstate = THROW_ACCELERATE;
                if (throwhsm.throwstate == THROW_ACCELERATE && caldata.angle>=22500.0)  throwhsm.throwstate = THROW_THROWOUT;
                if (throwhsm.throwstate == THROW_THROWOUT && (hDJI[1].Calculate.RotorRound > -1 || caldata.angle >= 32000.0))  throwhsm.throwstate = THROW_BACK;
            }
            //拍球状态修改
            else if (throwhsm.wholestate == WHOLE_BOUNCE)
            // else if(1)
            {
                if      (throwhsm.bouncestate == BOUNCE_IDLE && rmctl.rmctl_msg.btn_Btn0 == 1)  throwhsm.bouncestate = BOUNCE_CATCHING;
                else if (throwhsm.bouncestate == BOUNCE_CATCHING && rmctl.rmctl_msg.btn_Btn1 == 1)  throwhsm.bouncestate = BOUNCE_GETTOPOSITION;
                else if (throwhsm.bouncestate == BOUNCE_GETTOPOSITION && rmctl.rmctl_msg.btn_Btn2 == 1)  throwhsm.bouncestate = BOUNCE_READY;
                else if (throwhsm.bouncestate == BOUNCE_READY && rmctl.rmctl_msg.btn_Btn3 == 1)  throwhsm.bouncestate = BOUNCE_BOUNCE;
                else if ((throwhsm.bouncestate == BOUNCE_BOUNCE && externFlag_startSensor == 1) &&(rmctl.rmctl_msg.btn_LeftCrossDown ||deltime == TimeUp))
                {
                    throwhsm.bouncestate = BOUNCE_CATCHANDADJUSTPOSTURE;
                    deltime = 0;
                }
                else if (throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE && rmctl.rmctl_msg.btn_RightCrossUp ==1) throwhsm.bouncestate = BOUNCE_ADJUSTPOSTURE;
                else if (throwhsm.bouncestate == BOUNCE_ADJUSTPOSTURE && rmctl.rmctl_msg.btn_RightCrossUp ==1) throwhsm.bouncestate = BOUNCE_CATCHANDADJUSTPOSTURE;
                else if (throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE && rmctl.rmctl_msg.btn_Btn3 == 1)  throwhsm.bouncestate = BOUNCE_BOUNCE;
                else if (throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE && rmctl.rmctl_msg.btn_Btn4 == 1)  throwhsm.bouncestate = BOUNCE_WAITBALL;
                else if (throwhsm.bouncestate == BOUNCE_WAITBALL && rmctl.rmctl_msg.btn_LeftCrossUp == 1)  throwhsm.bouncestate = BOUNCE_CATCHANDADJUSTPOSTURE;
                else if ((throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE || throwhsm.bouncestate == BOUNCE_READY) && rmctl.rmctl_msg.btn_Btn5 == 1)  throwhsm.bouncestate = BOUNCE_READYTHROW;
            }
            //连续拍球状态修改
            else if (throwhsm.wholestate == WHOLE_CONBOUNCE)
            {
                if      (throwhsm.bouncestate == BOUNCE_IDLE && rmctl.rmctl_msg.btn_Btn0 == 1)  throwhsm.bouncestate = BOUNCE_CATCHING;
                else if (throwhsm.bouncestate == BOUNCE_CATCHING && rmctl.rmctl_msg.btn_Btn1 == 1)  throwhsm.bouncestate = BOUNCE_GETTOPOSITION;
                else if (throwhsm.bouncestate == BOUNCE_GETTOPOSITION && rmctl.rmctl_msg.btn_Btn2 == 1)  throwhsm.bouncestate = BOUNCE_READY;
                else if(throwhsm.bouncestate == BOUNCE_READY || throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE)
                {
                    if(rmctl.rmctl_msg.btn_RightCrossLeft == 1)     TimeUp = 80;
                    else if(rmctl.rmctl_msg.btn_RightCrossDown == 1)   TimeUp = 90;
                    else if(rmctl.rmctl_msg.btn_RightCrossMid == 1)   TimeUp = 100;
                    else if(rmctl.rmctl_msg.btn_RightCrossRight == 1)  TimeUp = 110;
                }
                else if (throwhsm.bouncestate == BOUNCE_READY && onceflag2 == 1)  
                {
                    throwhsm.bouncestate = BOUNCE_BOUNCE;
                    onceflag2 = 0;
                }
                else if ((throwhsm.bouncestate == BOUNCE_BOUNCE && externFlag_startSensor == 1) &&(rmctl.rmctl_msg.btn_LeftCrossDown ||  deltime == TimeUp))
                {
                    throwhsm.bouncestate = BOUNCE_CATCHANDADJUSTPOSTURE;
                    deltime = 0;
                }
                else if (throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE && onceflag1 ==1) throwhsm.bouncestate = BOUNCE_ADJUSTPOSTURE,onceflag1 = 0;
                else if (throwhsm.bouncestate == BOUNCE_ADJUSTPOSTURE && onceflag1 ==1) throwhsm.bouncestate = BOUNCE_CATCHANDADJUSTPOSTURE,onceflag1 = 0;
                else if (throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE && onceflag2 == 1)  throwhsm.bouncestate = BOUNCE_BOUNCE,onceflag2=0;
                else if ((throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE || throwhsm.bouncestate == BOUNCE_READY)&& rmctl.rmctl_msg.btn_Btn5 == 1)  throwhsm.bouncestate = BOUNCE_WAITBALL;
                else if (throwhsm.bouncestate == BOUNCE_WAITBALL && rmctl.rmctl_msg.btn_LeftCrossUp == 1)  throwhsm.bouncestate = BOUNCE_CATCHANDADJUSTPOSTURE;
            }
        }
        if(rmctl.rmctl_msg.btn_RightCrossUp == 1 && risedownflag1 == 0)
        {
            risedownflag1 = 1;
            onceflag1 = 1;
        }
        else if(rmctl.rmctl_msg.btn_RightCrossUp == 0 && risedownflag1 == 1)
        {
            risedownflag1 = 0;
        }
        if(rmctl.rmctl_msg.btn_Btn4 == 1 && risedownflag2 == 0)
        {
            risedownflag2 = 1;
            onceflag2 = 1;
        }
        else if(rmctl.rmctl_msg.btn_Btn4 == 0 && risedownflag2 == 1)
        {
            risedownflag2 = 0;
        }
        osDelay(2);
    }
}