#pragma once
#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <zlgcan/zlgcan.h>
#include "feedback.h"

#include <string>
#include <thread>
#include <iomanip>            // 用于格式化输入输出
#include <mutex> 
#include <condition_variable>
#include "MotorController.h"






class MotorRunner {
public:
    // 构造函数使用初始化列表来初始化 motor_ctrl
    MotorRunner(UINT  DevType1, UINT  DevIndex1, UINT  CanIndex1)
        : motor_ctrl0(new MotorController(DevType1, DevIndex1, CanIndex1)) 
        , motor_ctrl1(new MotorController(DevType1, DevIndex1, ++CanIndex1)) 
        , motor_ctrl2(new MotorController(DevType1, DevIndex1, ++CanIndex1))
        , motor_ctrl3(new MotorController(DevType1, DevIndex1, ++CanIndex1)){
            DevType = DevType1;
            DevIndex = DevIndex1;
            CanIndex = CanIndex1;
        }

    void run();
    int OpenDevice();

    //云深处电机接口
    void MotorRun();
    void Deepsend();
    void Deepreceive();
    
    MotorController* motor_ctrl0;
    MotorController* motor_ctrl1;
    MotorController* motor_ctrl2;
    MotorController* motor_ctrl3;
    int circulate_count = 0;

    UINT  DevType;
    UINT  DevIndex;
    UINT  CanIndex;
    UINT Reserved = 0;
    DEVICE_HANDLE  dHandle;
    CHANNEL_HANDLE cHandle;
    ZCAN_DEVICE_INFO  dinfo;
    ZCAN_CHANNEL_ERR_INFO einfo;
    IProperty *property;

    //以下为位置环测试用参数,给输入一个正弦角度
    float amplitude = M_PI;
    float frequency = 1.0; // 正弦波的频率
    float phase = 0.0;     // 初始相位
    float sampling_rate = 0.001; // 采样率，单位为秒
    int duration = 5;      // 持续时间，单位为秒
    float t = 0.0; // 初始化时间变量

private:
    void send();
    void receive();
    std::mutex mtx;
    std::condition_variable cv;
    bool data_ready = false;

   
};
