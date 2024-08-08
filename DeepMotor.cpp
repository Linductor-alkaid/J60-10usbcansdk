#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <zlgcan/zlgcan.h>
#include "unistd.h"
#include <stdio.h>
#include "enable.h"
#include "MotorController.h"
#include "MotorRunner.h"
#include "PeriodicTask.h"
#include <csignal>

UINT  DevType = ZCAN_USBCAN_4E_U;
UINT  DevIndex = 0x0;
UINT  CanIndex = 0;

MotorRunner* motorRunner = new MotorRunner(DevType, DevIndex, CanIndex);


std::ofstream log_file("MotorDatas.csv"); // 创建日志文件

void motorrun() {
    auto send_start = std::chrono::high_resolution_clock::now();
    
    motorRunner->MotorRun();

    auto send_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> send_duration = send_end - send_start;
    std::cout << "发送时间：" << send_duration.count() << std::endl;
    // std::ofstream log_file("MotorDatas.csv"); // 创建日志文件
    log_file << motorRunner->circulate_count << "," << motorRunner->motor_ctrl0->_mf.position << "," << motorRunner->motor_ctrl0->_mf.velocity << "," << motorRunner->motor_ctrl0->_mf.torque << "," << send_duration.count() << std::endl;
    // log_file.close(); // 关闭日志文件

}


void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    motorRunner->motor_ctrl0->disableDeepMotor();
    log_file.close(); // 关闭日志文件
    // 结束程序
    exit(signum);
}


int main(int argc, char *argv[]){
    // std::cout << DevType << std::endl;

    PeriodicTaskManager taskManager;
    PeriodicMemberFunction MotorTask(&taskManager, 0.002, "motor_thread", &motorrun);

    motorRunner->OpenDevice();

    motorRunner->motor_ctrl0->init();
    motorRunner->motor_ctrl0->enableDeepMotor();
    // motorRunner->motor_ctrl1->init();
    // motorRunner->motor_ctrl2->init();
    // motorRunner->motor_ctrl3->init();

    // sleep(5000);

    signal(SIGINT, signalHandler); // 注册信号处理函数

    MotorTask.start();
    while(true);
    // motorRunner->motor_ctrl0->disableDeepMotor();

    // while(1){
    //     // 记录发送开始时间
    //     auto send_start = std::chrono::high_resolution_clock::now();
    // motorRunner->run();
    // usleep(500);
    // // 记录发送结束时间
    // auto send_end = std::chrono::high_resolution_clock::now();
    // // 计算发送时间
    // std::chrono::duration<double> send_duration = send_end - send_start;
    // std::cout << "发送时间：" << send_duration.count() << std::endl;
    // // ZCAN_ClearBuffer(motorRunner0->motor_ctrl->channel_handle);
    // }
    return 0;
}