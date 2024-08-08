#include <zlgcan/zlgcan.h>
#include "feedback.h"

#include <string>
#include <thread>
#include <iomanip>            // 用于格式化输入输出
#include <chrono>             // 用于时间和定时功能
#include <fstream>            // 用于文件输入输出
#include <mutex> 
#include <condition_variable>
#include <cmath>

#include "MotorRunner.h"


int MotorRunner::OpenDevice(){
    dHandle = ZCAN_OpenDevice(DevType, DevIndex, Reserved);

    if (dHandle == INVALID_DEVICE_HANDLE) {
        printf("ZCAN_OpenDevice failed\n");
        return -1;
    }

    motor_ctrl0->dHandle = dHandle;
    motor_ctrl1->dHandle = dHandle;
    motor_ctrl2->dHandle = dHandle;
    motor_ctrl3->dHandle = dHandle;
    return 0;
}


void MotorRunner::run() {
    std::thread send_thread(&MotorRunner::send, this);
    std::thread receive_thread(&MotorRunner::receive, this);

    send_thread.join();
    receive_thread.join();
}

void MotorRunner::MotorRun() {
    // auto send_start = std::chrono::high_resolution_clock::now();

    std::thread send_thread(&MotorRunner::Deepsend, this);
    std::thread receive_thread(&MotorRunner::Deepreceive, this);

    send_thread.join();
    receive_thread.join();

    // Deepsend();
    // Deepreceive();

    // auto send_end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> send_duration = send_end - send_start;
    // std::cout << "发送时间：" << send_duration.count() << std::endl;
    // std::ofstream log_file("MotorDatas.csv"); // 创建日志文件
    // log_file << circulate_count << "," << 5 << "," << motor_ctrl0->_mf.velocity << "," << motor_ctrl0->_mf.torque << "," << send_duration.count() << std::endl;
    // log_file.close(); // 关闭日志文件
}

void MotorRunner::send() {
    std::unique_lock<std::mutex> lock(mtx);
    
    motor_ctrl0->sendTorqueToServer(1, 0);
    motor_ctrl0->sendTorqueToServer(2, 0);   
    motor_ctrl0->sendTorqueToServer(3, 0);
    motor_ctrl1->sendTorqueToServer(8, 0);
    motor_ctrl1->sendTorqueToServer(9, 0);
    motor_ctrl1->sendTorqueToServer(10, 0);
    motor_ctrl2->sendTorqueToServer(1, 0);
    motor_ctrl2->sendTorqueToServer(2, 0);   
    motor_ctrl2->sendTorqueToServer(3, 0);
    motor_ctrl3->sendTorqueToServer(8, 0);
    motor_ctrl3->sendTorqueToServer(9, 0);
    motor_ctrl3->sendTorqueToServer(10, 0);

    data_ready = true;
    cv.notify_all();
}


void MotorRunner::receive() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]{ return data_ready; });

    motor_ctrl0->receiveDataFromServer();
    // motor_ctrl1->receiveDataFromServer();
    // motor_ctrl2->receiveDataFromServer();
    // motor_ctrl3->receiveDataFromServer();

    std::cout << "" << circulate_count << std::endl;

    // if (circulate_count == 6) {
    //     circulate_count = 0;

    //     ZCAN_ClearBuffer(motor_ctrl0->channel_handle);
    //     // ZCAN_ClearBuffer(motor_ctrl1->channel_handle);
    //     // ZCAN_ClearBuffer(motor_ctrl2->channel_handle);
    //     // ZCAN_ClearBuffer(motor_ctrl3->channel_handle);

    //     std::cout << "sngg";
    // }

    // circulate_count++;
}


void MotorRunner::Deepsend(){
    std::unique_lock<std::mutex> lock(mtx);
    
    // motor_ctrl0->sendTorqueToDeepMotor(1, 5);
    // motor_ctrl0->sendPositionToDeepMotor(1, -3, 20, 10);
    // motor_ctrl0->sendVelocityToDeepMotor(1, 5, 5);

    // motor_ctrl0->sendTorqueToDeepMotor(2, 0);   
    // motor_ctrl0->sendTorqueToDeepMotor(3, 0);
    // motor_ctrl1->sendTorqueToDeepMotor(8, 0);
    // motor_ctrl1->sendTorqueToDeepMotor(9, 0);
    // motor_ctrl1->sendTorqueToDeepMotor(10, 0);
    // motor_ctrl2->sendTorqueToDeepMotor(1, 0);
    // motor_ctrl2->sendTorqueToDeepMotor(2, 0);   
    // motor_ctrl2->sendTorqueToDeepMotor(3, 0);
    // motor_ctrl3->sendTorqueToDeepMotor(8, 0);
    // motor_ctrl3->sendTorqueToDeepMotor(9, 0);
    // motor_ctrl3->sendTorqueToDeepMotor(10, 0);

    float p = amplitude * sin(2 * M_PI * frequency * t + phase);

    // std::cout << p << std::endl;

    motor_ctrl0->sendPositionToDeepMotor(1, p, 30, 6);
    
    t += 0.002;

    data_ready = true;
    cv.notify_all();
}


void MotorRunner::Deepreceive() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]{ return data_ready; });

    motor_ctrl0->receiveDataFromDeepMotor();
    // motor_ctrl1->receiveDataFromServer();
    // motor_ctrl2->receiveDataFromServer();
    // motor_ctrl3->receiveDataFromServer();
    
    std::cout << "" << circulate_count << std::endl;

    // if (circulate_count == 6) {
    //     circulate_count = 0;

    //     ZCAN_ClearBuffer(motor_ctrl0->channel_handle);
    //     // ZCAN_ClearBuffer(motor_ctrl1->channel_handle);
    //     // ZCAN_ClearBuffer(motor_ctrl2->channel_handle);
    //     // ZCAN_ClearBuffer(motor_ctrl3->channel_handle);

    //     std::cout << "sngg";
    // }

    circulate_count++;
}