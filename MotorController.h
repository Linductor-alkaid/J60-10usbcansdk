#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H


#include <iostream>
#include <cmath>
#include <zlgcan/zlgcan.h>
#include "feedback.h"

#include <string>
#include <thread>
#include <iomanip>            // 用于格式化输入输出
#include <chrono>             // 用于时间和定时功能
#include <fstream>            // 用于文件输入输出
#include <mutex> 
#include <condition_variable>
#include "can_protocol.h"

//用于存储发向电机的数据
typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    float kp_;
    float kd_;
}MotorCMD;


//用于存储电机返回的数据
typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    bool flag_;
    float temp_;
    uint16_t error_;
}MotorDATA;


enum MotorErrorType{
    //*******************************
    //MotorErrorType: SendRecv函数的返回值
    //*******************************
    //全为0: 无错误
    //bit 0: 过压标志位
    //bit 1: 欠压标志位
    //bit 2: 过流标志位
    //bit 3: 关节过温标志位
    //bit 4: 驱动板过温标志位
    //bit 5: Can超时标志位
    kMotorNoError = 0,
    kOverVoltage = (0x01 << 0),
    kUnderVoltage = (0x01 << 1),
    kOverCurrent = (0x01 << 2),
    kMotorOverTemp = (0x01 << 3),
    kDriverOverTemp = (0x01 << 4),
    kCanTimeout = (0x01 << 5)
};


class MotorController {
public:
    // 构造函数，初始化设备类型、设备索引和CAN通道索引
    MotorController(UINT DevType1, UINT DevIndex1, UINT CanIndex1){
        // std::cout << CanIndex1 << std::endl;
        DevType = DevType1;
        DevIndex = DevIndex1;
        CanIndex = CanIndex1;
    };
    ~MotorController(){};

    motor_feedback _mf; 
    DEVICE_HANDLE device_handle;
    CHANNEL_HANDLE channel_handle;


    int init();
    bool connectToServer();
    void enableMotor();
    void disableMotor();
    void sendTorqueToServer(unsigned char id, float set_torque);
    void sendPositionToServer(unsigned char id, float set_position, float kp, float kd);
    void sendVelocityToServer(unsigned char id, float set_velocity, float kd);
    void receiveDataFromServer();
    void keeper();

    //兼容云深处

    void enableDeepMotor();
    void disableDeepMotor();
    void FloatsToUints(const MotorCMD *param, uint8_t *data);
    void UintsToFloats(const struct can_frame *frame, MotorDATA *data);
    void ParseRecvFrame(const struct can_frame *frame_ret, MotorDATA *data);
    void sendTorqueToDeepMotor(unsigned char id, float set_torque);
    void sendPositionToDeepMotor(unsigned char id, float set_position, float kp, float kd);
    void sendVelocityToDeepMotor(unsigned char id, float set_velocity, float kd);
    void receiveDataFromDeepMotor();
    
    UINT Reserved = 0;
    DEVICE_HANDLE  dHandle;
    CHANNEL_HANDLE cHandle;
    ZCAN_DEVICE_INFO  dinfo;
    ZCAN_CHANNEL_ERR_INFO einfo;
    IProperty *property;

private:
    UINT DevType;
    UINT DevIndex;
    UINT CanIndex;
    float pos,veloc,tor;
};


#endif // MOTORCONTROLLER_H