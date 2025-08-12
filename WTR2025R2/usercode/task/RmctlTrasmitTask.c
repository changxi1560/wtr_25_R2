/*
 * @Description: 遥控器消息传递线程
 * @Author: Alex
 * @Date: 2025-03-28 20:59:22
 * @LastEditors: Alex
 * @LastEditTime: 2025-03-28 21:05:30
 */

#include "RmctlTrasmitTask.h"
#include "usart.h"

void RmctlTrasmitTask(void* argument)
{
    mavlink_joystick_air_dashboard_set_title_t  mavlink_wholestate_title;
    mavlink_joystick_air_dashboard_set_msg_t    mavlink_wholestate_msg;
    mavlink_joystick_air_dashboard_set_title_t  mavlink_throwstate_title;
    mavlink_joystick_air_dashboard_set_msg_t    mavlink_throwstate_msg;
    mavlink_joystick_air_dashboard_set_title_t  mavlink_bouncestate_title;
    mavlink_joystick_air_dashboard_set_msg_t    mavlink_bouncestate_msg;
    mavlink_joystick_air_dashboard_set_title_t  mavlink_errorstate_title;
    mavlink_joystick_air_dashboard_set_msg_t    mavlink_errorstate_msg;

    int state[22] = {0};
    for (;;)
    {
      /*
      if(rmctl.rmctl_msg.btn_Btn0 == 1) state[0] = 1;
      if(rmctl.rmctl_msg.btn_Btn1 == 1) state[1] = 1; 
      if(rmctl.rmctl_msg.btn_Btn2 == 1) state[2] = 1; 
      if(rmctl.rmctl_msg.btn_Btn3 == 1) state[3] = 1; 
      if(rmctl.rmctl_msg.btn_Btn4 == 1) state[4] = 1; 
      if(rmctl.rmctl_msg.btn_Btn5 == 1) state[5] = 1; 
      if(rmctl.rmctl_msg.btn_JoystickL == 1) state[6] = 1; 
      if(rmctl.rmctl_msg.btn_JoystickR == 1) state[7] = 1; 
      if(rmctl.rmctl_msg.btn_KnobL == 1) state[8] = 1; 
      if(rmctl.rmctl_msg.btn_KnobR == 1) state[9] = 1; 
      if(rmctl.rmctl_msg.btn_LeftCrossDown == 1) state[10] = 1; 
      if(rmctl.rmctl_msg.btn_LeftCrossLeft == 1) state[11] = 1; 
      if(rmctl.rmctl_msg.btn_LeftCrossMid == 1) state[12] = 1; 
      if(rmctl.rmctl_msg.btn_LeftCrossRight == 1) state[13] = 1; 
      if(rmctl.rmctl_msg.btn_LeftCrossUp == 1) state[14] = 1; 
      if(rmctl.rmctl_msg.btn_RightCrossDown == 1) state[15] = 1; 
      if(rmctl.rmctl_msg.btn_RightCrossLeft == 1) state[16] = 1; 
      if(rmctl.rmctl_msg.btn_RightCrossMid == 1) state[17] = 1; 
      if(rmctl.rmctl_msg.btn_RightCrossRight == 1) state[18] = 1; 
      if(rmctl.rmctl_msg.btn_RightCrossUp == 1) state[19] = 1; 
      if(rmctl.rmctl_msg.left_switch== 1) state[20] = 1; 
      if(rmctl.rmctl_msg.right_switch == 1) state[21] = 1;
      

    if(rmctl.rmctl_msg.btn_Btn0 == 1)       HAL_UART_Transmit(&huart8,(uint8_t *)state,22,0xffff);
    osDelay(20);
    */
        //修改遥控器信息
        
        JoystickSwitchTitle(2, "WholeState", &mavlink_wholestate_title);
        JoystickSwitchTitle(3, "ThrowState", &mavlink_throwstate_title);
        JoystickSwitchTitle(4, "BounceState", &mavlink_bouncestate_title);
        JoystickSwitchTitle(5, "ErrorState", &mavlink_errorstate_title);
        

        if      (throwhsm.wholestate == WHOLE_INITING)                  JoystickSwitchMsg(2, "Initing", &mavlink_wholestate_msg);
        else if (throwhsm.wholestate == WHOLE_READY)                    JoystickSwitchMsg(2, "Ready", &mavlink_wholestate_msg);
        else if (throwhsm.wholestate == WHOLE_BOUNCE)                   JoystickSwitchMsg(2, "Bounce", &mavlink_wholestate_msg);
        else if (throwhsm.wholestate == WHOLE_CONBOUNCE)                JoystickSwitchMsg(2, "ConBounce", &mavlink_wholestate_msg);
        else if (throwhsm.wholestate == WHOLE_THROW)                    JoystickSwitchMsg(2, "Throw", &mavlink_wholestate_msg);
        else if (throwhsm.wholestate == WHOLE_CONTHROW)                 JoystickSwitchMsg(2, "ConThrow", &mavlink_wholestate_msg);
        else if (throwhsm.wholestate == WHOLE_ERROR)                    JoystickSwitchMsg(2, "Error", &mavlink_wholestate_msg);

        if      (throwhsm.bouncestate == BOUNCE_IDLE)                   JoystickSwitchMsg(4, "Idle", &mavlink_bouncestate_msg);
        else if (throwhsm.bouncestate == BOUNCE_WAITBALL)               JoystickSwitchMsg(4, "WaitBall", &mavlink_bouncestate_msg);
        else if (throwhsm.bouncestate == BOUNCE_CATCHING)               JoystickSwitchMsg(4, "Catching", &mavlink_bouncestate_msg);
        else if (throwhsm.bouncestate == BOUNCE_GETTOPOSITION)          JoystickSwitchMsg(4, "GetToPositon", &mavlink_bouncestate_msg);
        else if (throwhsm.bouncestate == BOUNCE_READY)                  JoystickSwitchMsg(4, "Ready", &mavlink_bouncestate_msg);
        else if (throwhsm.bouncestate == BOUNCE_BOUNCE)                 JoystickSwitchMsg(4, "Bounce", &mavlink_bouncestate_msg);
        else if (throwhsm.bouncestate == BOUNCE_CATCHANDADJUSTPOSTURE)  JoystickSwitchMsg(4, "CatchAndAdjustPosture", &mavlink_bouncestate_msg);

        if      (throwhsm.throwstate == THROW_IDLE)                     JoystickSwitchMsg(3, "Idle", &mavlink_throwstate_msg);
        else if (throwhsm.throwstate == THROW_GATHERSTRENGTH)           JoystickSwitchMsg(3, "GatherStrength", &mavlink_throwstate_msg);
        else if (throwhsm.throwstate == THROW_CLEARBRACE)               JoystickSwitchMsg(3, "ClearBrace", &mavlink_throwstate_msg);
        else if (throwhsm.throwstate == THROW_ACCELERATE)               JoystickSwitchMsg(3, "Acclerate", &mavlink_throwstate_msg);
        else if (throwhsm.throwstate == THROW_THROWOUT)                 JoystickSwitchMsg(3, "ThrowOut", &mavlink_throwstate_msg);
        else if (throwhsm.throwstate == THROW_BACK)                     JoystickSwitchMsg(3, "Back", &mavlink_throwstate_msg);
        else if (throwhsm.throwstate == THROW_CATCHING)                 JoystickSwitchMsg(3, "Catching", &mavlink_throwstate_msg);
        
        if      (throwhsm.errorstate == ERROR_CANINITFAIL1)             JoystickSwitchMsg(5, "Can1Fail", &mavlink_errorstate_msg);
        else if (throwhsm.errorstate == ERROR_CANINITFAIL2)             JoystickSwitchMsg(5, "Can2Fail", &mavlink_errorstate_msg);
        else if (throwhsm.errorstate == ERROR_UNITREEINITFAIL0)         JoystickSwitchMsg(5, "Unitree0Fail", &mavlink_errorstate_msg);
        else if (throwhsm.errorstate == ERROR_UNITREEINITFAIL1)         JoystickSwitchMsg(5, "Unitree1Fail", &mavlink_errorstate_msg);
        else if (throwhsm.errorstate == ERROR_ECODERINITFAIL)           JoystickSwitchMsg(5, "EncoderFail", &mavlink_errorstate_msg);
        else if (throwhsm.errorstate == ERROR_SENSORINITFAIL)           JoystickSwitchMsg(5, "SensorFail", &mavlink_errorstate_msg);
        else if (throwhsm.errorstate == ERROR_DJIINITFAIL)              JoystickSwitchMsg(5, "DJIFail", &mavlink_errorstate_msg);
        else if (throwhsm.errorstate == ERROR_DJITRANSMITFAIL)          JoystickSwitchMsg(5, "DJIMegFail", &mavlink_errorstate_msg);
        else if (throwhsm.errorstate == ERROR_NONE)                     JoystickSwitchMsg(5, "None", &mavlink_errorstate_msg);
        
        osDelay(2);
        
    }
}