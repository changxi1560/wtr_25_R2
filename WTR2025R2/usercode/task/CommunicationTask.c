#include "CommunicationTask.h"

Com_t MyData;
uint16_t Txbuffer[5];
//uint16_t RxbufferFromDown[5];
uint8_t RxbufferFromUp[28];
float raw_camera[3];
float raw_lidar[3];
int Rxbuffer0[5] = {0};

void CommunicationTask(void* argument)
{
    PIDinit(&MyData);
    KalmanFilter2D_reset(&kf2dx,0.1);
    KalmanFilter2D_reset(&kf2dy,0.1);
    for(;;)
    {
        MyData.BasketPos.x = 4.03;
        MyData.BasketPos.y = 0.996;
        if(throwhsm.throwstate == THROW_RIDARPOSITIONSERVE)
        {
            MyData.TargetPos.x = MyData.MyPos.x;
            MyData.TargetPos.y = MyData.MyPos.y;
            MyData.TargetPos.angle = atan((MyData.BasketPos.y - MyData.TargetPos.y)/(MyData.BasketPos.x - MyData.TargetPos.x));

            AngleServe(&MyData,MyData.TargetPos.angle);
            PositionServe(&MyData,MyData.TargetPos.x,MyData.TargetPos.y);
        }
        else if(throwhsm.throwstate == THROW_CAMERAPOSITIONSERVE)
        {
            MyData.TargetPos.x = MyData.MyPos.x;
            MyData.TargetPos.y = MyData.MyPos.y;

            CameraServe(&MyData,0);
            PositionServe(&MyData,MyData.TargetPos.x,MyData.TargetPos.y);
        }
        DataTransfer(&MyData);
        osDelay(10);
    }
}

void AngleServe(Com_t * Data,double Target)  //角度伺服
{
    Data->AnglePID.ref = Target;
    Data->AnglePID.fdb = Data->MyPos.angle;
    PID_Cal(&Data->AnglePID);
}

void CameraServe(Com_t * Data,double Target) //使篮筐位于相机横坐标某点的伺服
{
    Data->CameraPID.ref = Target;
    Data->CameraPID.fdb = Data->camera_basket_xyz.x;
    PID_Cal(&Data->CameraPID);
}

void PositionServe(Com_t * Data,double TargetX,double TargetY)//直角坐标位置伺服
{
    Data->xPosPID.ref = TargetX;
    Data->xPosPID.fdb = Data->MyPos.x;
    Data->yPosPID.ref = TargetY;
    Data->yPosPID.fdb = Data->MyPos.y;
    PID_Cal(&Data->xPosPID);
    PID_Cal(&Data->yPosPID);
}

void PID_Cal(PID *pid)
{
	pid->cur_error = pid->ref - pid->fdb;
	pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
	pid->error[0] = pid->error[1];
	pid->error[1] = pid->ref - pid->fdb;
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;

}


uint16_t ComputeCRC16(uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; ++j)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void DataTransfer(Com_t * Data) //上位机与底盘的收发函数
{
    if(HAL_UART_Receive(&huart2,(uint8_t *)RxbufferFromUp,sizeof(RxbufferFromUp),0xffff) == HAL_OK) //正常接收数据时
    {
        if(RxbufferFromUp[0] == 0xAA && RxbufferFromUp[1] == 0x55) //判断帧头后解包
        {
            uint16_t received_crc;
            memcpy(&received_crc,RxbufferFromUp + 26, 2);
            uint16_t computed_crc = ComputeCRC16(RxbufferFromUp, 26);
            if (received_crc == computed_crc)   //判断校验位
            {
                memcpy(&raw_lidar[0], RxbufferFromUp + 2, 4);
                memcpy(&raw_lidar[1], RxbufferFromUp + 6, 4);
                memcpy(&raw_lidar[2], RxbufferFromUp + 10, 4);
                memcpy(&raw_camera[0] , RxbufferFromUp + 14 , 4);
                memcpy(&raw_camera[1] , RxbufferFromUp + 18 , 4);
                memcpy(&raw_camera[2], RxbufferFromUp + 22 , 4);

                Data->MyPos.x = KalmanFilter2D_calculate(&kf2dx, raw_lidar[0]);
                Data->MyPos.y = KalmanFilter2D_calculate(&kf2dy, raw_lidar[1]);
                Data->MyPos.angle = raw_lidar[2];
                Data->camera_basket_xyz.x = raw_camera[0];
                Data->MyPos.angle = Data->MyPos.angle /90.0f *M_PI_2;
            }



            if(throwhsm.throwstate == THROW_RIDARPOSITIONSERVE)  //位置矫正时向底盘发送数据
            {
                Txbuffer[0] = 0xFF;
                Txbuffer[4] = 0xFE;
                Txbuffer[1] = (uint16_t)((Data->xPosPID.output*cos(Data->MyPos.angle)+Data->yPosPID.output*sin(Data->MyPos.angle))*100);
                Txbuffer[2] = (uint16_t)((Data->xPosPID.output*sin(Data->MyPos.angle)-Data->yPosPID.output*cos(Data->MyPos.angle))*100);
                Txbuffer[3] = (uint16_t)(Data->AnglePID.output*100);
                if(HAL_UART_Transmit(&huart8,(uint8_t *)Txbuffer,sizeof(Txbuffer),0xffff) != HAL_OK) //不正常发送数据时
                {
                    throwhsm.errorstate = ERROR_COMMUNICATIONFAIL;
                }            
            }
            else if(throwhsm.throwstate == THROW_CAMERAPOSITIONSERVE)
            {
                Txbuffer[0] = 0xFF;
                Txbuffer[4] = 0xFE;
                Txbuffer[1] = (uint16_t)((Data->xPosPID.output*cos(Data->MyPos.angle)+Data->yPosPID.output*sin(Data->MyPos.angle))*100);
                Txbuffer[2] = (uint16_t)((Data->xPosPID.output*sin(Data->MyPos.angle)-Data->yPosPID.output*cos(Data->MyPos.angle))*100);
                Txbuffer[3] = (uint16_t)(Data->CameraPID.output*100);
                if(HAL_UART_Transmit(&huart8,(uint8_t *)Txbuffer,sizeof(Txbuffer),0xffff) != HAL_OK) //不正常发送数据时
                {
                    throwhsm.errorstate = ERROR_COMMUNICATIONFAIL;
                }            
            }

        }
        else
        {
            throwhsm.errorstate = ERROR_COMMUNICATIONFAIL;
        }
    }
    else
    {
        throwhsm.errorstate = ERROR_COMMUNICATIONFAIL;
    }
}


void PIDinit(Com_t *Data)
{
    Data->AnglePID.KP = 2;
    Data->AnglePID.KI = 0.4;
    Data->AnglePID.KD = 0.4;

    Data->xPosPID.KP = 2;
    Data->xPosPID.KI = 0.4;
    Data->xPosPID.KD = 0.4;

    Data->yPosPID.KP = 2;
    Data->yPosPID.KI = 0.4;
    Data->yPosPID.KD = 0.4;

    Data->CameraPID.KP = 2;
    Data->CameraPID.KI = 0.4;
    Data->CameraPID.KD = 0.4;
}

