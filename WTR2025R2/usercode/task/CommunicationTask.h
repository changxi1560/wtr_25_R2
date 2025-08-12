#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "AllUserTask.h"
#include "usart.h"
#define _USE_MATH_DEFINES
#include "math.h"
#include "ThrowHSM.h"
#include "Laser.h"
#include "circular_buffer.h"
#include "string.h"
#include "stdio.h"
#include "kalman_2dfilter.h"




    typedef struct
    {
        double x;        //x坐标
        double y;        //y坐标
        double angle;    //偏航角
    } Position;

    typedef struct 
    {
        double Xspeed; //X轴方向速度
        double Yspeed; //Y轴方向速度
        double w;      //角速度
    } Speed;
    

    typedef struct
    {
        float KP;        // PID参数P
        float KI;        // PID参数I
        float KD;        // PID参数D
        float fdb;       // PID反馈值
        float ref;       // PID目标值
        float cur_error; // 当前误差
        float error[2];  // 前两次误差
        float output;    // 输出值
        float outputMax; // 最大输出值的绝对值
        float outputMin; // 最小输出值的绝对值用于防抖
        float integral;  // 积分值
        float integralMax; // 积分最大值
        float integralMin; // 积分最小值
    } PID;

    typedef struct 
    {

        struct {
            double x ;
            double y ;
        }BasketPos;

        struct{
            double x;
            double y;
            double z;
        }camera_basket_xyz;
        Position MyPos;  
        Position TargetPos;
        Position DT35Pos;

        Speed MySpeed;
        Speed TargetSpeed;



        PID xPosPID;
        PID yPosPID;
        PID AnglePID;
        PID CameraPID;
    }Com_t;

    extern Com_t MyData;
    extern uint16_t Txbuffer[5];
    extern uint8_t RxbufferFromUp[28];
    extern float raw_lidar[3];
    extern float raw_camera[3];
    extern int Rxbuffer0[5];


    void CommunicationTask(void* argument);
    void AngleServe(Com_t * Communication,double Target);
    void PositionServe(Com_t * Communication,double TargetX,double TargetY);
    void PID_Cal(PID *pid);
    void DataTransfer(Com_t * Data);
    uint16_t ComputeCRC16(uint8_t* data, uint16_t length);
    void CameraServe(Com_t * Data,double Target);
    void PIDinit(Com_t *Data);


#endif