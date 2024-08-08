#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>
#include <zlgcan/zlgcan.h>
#include "feedback.h"

#include <string>
#include <thread>
#include <iomanip>            // 用于格式化输入输出
#include <chrono>             // 用于时间和定时功能
#include <fstream>            // 用于文件输入输出
#include <mutex> 
#include <condition_variable>
#include "MotorController.h"



bool MotorController::connectToServer() {
    
    int ret;
    char path[128];
    int i;

    // std::cout << DevType << std::endl;

    property = GetIProperty(dHandle);
    
    cHandle = ZCAN_InitCAN(dHandle, CanIndex, NULL);
    if (cHandle == INVALID_CHANNEL_HANDLE) {
        printf("ZCAN_InitCAN failed\n");
        ZCAN_CloseDevice(dHandle);
        return -1;
    }
	ZCAN_ResetCAN(cHandle);
    snprintf(path, 128, "info/channel/channel_%d/baud_rate", CanIndex);
    property->SetValue(path, "1000000");

    snprintf(path, 128, "info/channel/channel_%d/work_mode", CanIndex);
    property->SetValue(path, "0");

    ret = ZCAN_StartCAN(cHandle);
    if (ret < 0) {
        printf("ZCAN_StartCAN failed\n");
        ZCAN_CloseDevice(dHandle);
        return -1;
    }

    std::cout << "CAN INIT SUCCESS" << std::endl;

    return 1;

}


int MotorController::init(){

    if (!connectToServer()) {
        std::cerr << "无法连接到CAN总线" << std::endl;
        return -1;
    }
    
    device_handle = dHandle;
    channel_handle = cHandle;
    enableMotor();
    


    // RecTask.start();

    return 0;
}

void MotorController::enableMotor(){
    for (int i=1;i<12;i++){
    unsigned char data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    can_frame canObj;
    canObj.can_id = i;
    canObj.can_dlc = 8;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);
    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;


    if (ZCAN_Transmit(channel_handle, &Transmit_Data, 1) == STATUS_ERR) {
        std::cerr << "发送数据失败,电机ID:" << static_cast<int>(i) << std::endl;
    }}
}

void MotorController::enableDeepMotor(){
    for (__u8 i=1;i<12;i++){
    // unsigned char data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    MotorCMD *cmd = (MotorCMD*)malloc(sizeof(MotorCMD));;
    unsigned char data[8];
    cmd->torque_ = 0;
    cmd->velocity_ = 0;
    cmd->position_ = 0;
    cmd->kd_ = 0;
    cmd->kp_ = 0;
    cmd->cmd_ = 2;
    FloatsToUints(cmd, data);

    can_frame canObj;
    canObj.can_id = (cmd->cmd_ << 5) | i;
    canObj.can_dlc = 0;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);
    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;


    if (ZCAN_Transmit(channel_handle, &Transmit_Data, 1) == STATUS_ERR) {
        std::cerr << "发送数据失败,电机ID:" << static_cast<int>(i) << std::endl;
    }}
}


void MotorController::disableDeepMotor(){
    for (__u8 i=1;i<12;i++){
    // unsigned char data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    MotorCMD *cmd = (MotorCMD*)malloc(sizeof(MotorCMD));;
    unsigned char data[8];
    cmd->torque_ = 0;
    cmd->velocity_ = 0;
    cmd->position_ = 0;
    cmd->kd_ = 0;
    cmd->kp_ = 0;
    cmd->cmd_ = 1;
    FloatsToUints(cmd, data);

    can_frame canObj;
    canObj.can_id = (cmd->cmd_ << 5) | i;
    canObj.can_dlc = 0;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);
    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;


    if (ZCAN_Transmit(channel_handle, &Transmit_Data, 1) == STATUS_ERR) {
        std::cerr << "发送数据失败,电机ID:" << static_cast<int>(i) << std::endl;
    }}
}


void MotorController::sendTorqueToServer(unsigned char id, float set_torque) {
    // 计算新的扭矩值映射
    int torque = static_cast<int>((set_torque + 48.0) * 4095 / 96.0);
    torque = std::max(0, std::min(4095, torque)); // 确保扭矩值在0到4095之间

    // 直接使用整数值提取高4位和低8位
    unsigned char torque_high4 = static_cast<unsigned char>((torque >> 8) & 0x0F);
    unsigned char torque_low8 = static_cast<unsigned char>(torque & 0xFF);

    // 构建数据包
    unsigned char data[8] = {0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, static_cast<unsigned char>((0x00 << 4) | torque_high4), torque_low8};

    can_frame canObj;
    canObj.can_id = id;
    canObj.can_dlc = 8;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);
    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;
    // std::cout << "can: " << canIndex << " id: " << int(id) <<  " torque: " << torque << std::endl;

    // 发送数据包
    int result = ZCAN_Transmit(channel_handle, &Transmit_Data, 1);
    if (result == -1) {
        std::cerr << "发送数据失败，电机ID：" << static_cast<int>(id) << std::endl;
    } 
}


void MotorController::sendPositionToServer(unsigned char id, float set_position, float kp, float kd) {
    // 计算新的位置值映射
    int position = static_cast<int>((set_position + 4.0 * M_PI) * 65535 / (8.0 * M_PI));
    position = std::max(0, std::min(65535, position)); // 确保位置值在0到65535之间

    // 提取位置值的高8位和低8位
    unsigned char position_high8 = static_cast<unsigned char>((position >> 8) & 0xFF);
    unsigned char position_low8 = static_cast<unsigned char>(position & 0xFF);

    // 计算kp值映射
    int kp_mapped = static_cast<int>(kp * 4095 / 500.0);
    kp_mapped = std::max(0, std::min(4095, kp_mapped)); // 确保kp值在0到4095之间

    // 提取kp值的高4位和低8位
    unsigned char kp_high4 = static_cast<unsigned char>((kp_mapped >> 8) & 0x0F);
    unsigned char kp_low8 = static_cast<unsigned char>(kp_mapped & 0xFF);

    // 计算kd值映射
    int kd_mapped = static_cast<int>(kd * 4095 / 100.0);
    kd_mapped = std::max(0, std::min(4095, kd_mapped)); // 确保kd值在0到4095之间

    // 提取kd值的高8位和低4位
    unsigned char kd_high8 = static_cast<unsigned char>((kd_mapped >> 4) & 0xFF);
    unsigned char kd_low4 = static_cast<unsigned char>(kd_mapped & 0x0F);

    // 构建数据包
    unsigned char data[8] = {
        position_high8, position_low8,
        0x7F, static_cast<unsigned char>((0xF0) | (kp_high4 & 0x0F)), // 速度值和kp高4位
        kp_low8,
        kd_high8,
        static_cast<unsigned char>((kd_low4 << 4) | 0x07), // kd低4位和0x7
        0xFF
    };

    can_frame canObj;
    canObj.can_id = id;
    canObj.can_dlc = 8;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);
    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;

    // 发送数据包
   int result = ZCAN_Transmit(channel_handle, &Transmit_Data, 1);
    if (result == -1) {
        std::cerr << "发送数据失败，电机ID：" << static_cast<int>(id) << std::endl;
    } 
}


void MotorController::sendVelocityToServer(unsigned char id, float set_velocity, float kd) {
    // 计算速度值映射
    int velocity = static_cast<int>((set_velocity + 30.0) * 4095 / 60.0);
    velocity = std::max(0, std::min(4095, velocity)); // 确保速度值在0到4095之间

    // 直接使用整数值提取高8位和低4位
    unsigned char velocity_high8 = static_cast<unsigned char>((velocity >> 4) & 0xFF);
    unsigned char velocity_low4 = static_cast<unsigned char>(velocity & 0x0F);

    // 计算kd值映射
    int kd_mapped = static_cast<int>(kd * 4095 / 100.0);
    kd_mapped = std::max(0, std::min(4095, kd_mapped)); // 确保kd值在0到4095之间

    // 直接使用整数值提取高8位和低4位
    unsigned char kd_high8 = static_cast<unsigned char>((kd_mapped >> 4) & 0xFF);
    unsigned char kd_low4 = static_cast<unsigned char>(kd_mapped & 0x0F);

    // 构建数据包
    unsigned char data[8] = {
        0x7F, 0xFF, 
        velocity_high8, static_cast<unsigned char>((velocity_low4 << 4) & 0xF0), 
        0x00, kd_high8, static_cast<unsigned char>(((kd_low4 << 4) & 0xF0) | 0x07), 
        0xFF
    };

    can_frame canObj;
    canObj.can_id = id;
    canObj.can_dlc = 8;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);
    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;

    // 发送数据包
   int result = ZCAN_Transmit(channel_handle, &Transmit_Data, 1);
    if (result == -1) {
        std::cerr << "发送数据失败，电机ID：" << static_cast<int>(id) << std::endl;
    } 
}


void MotorController::receiveDataFromServer() {
    ZCAN_Receive_Data received[10];
    int receiveCount = ZCAN_Receive(channel_handle, received, 3, 2);

    if (receiveCount > 0) {
        for (int i = 0; i < receiveCount; ++i) {
            unsigned short positionRaw = (received[i].frame.data[1] << 8) | received[i].frame.data[2];
            double position = (static_cast<unsigned short>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);
            unsigned short velocityRaw = (received[i].frame.data[3] << 4) | (received[i].frame.data[4] >> 4);
            double velocity = (static_cast<unsigned short>(velocityRaw) - 2047.0) / 2047.0 * 30 * M_PI;
            unsigned short torqueRaw = ((received[i].frame.data[4]&0x0F) << 8) | received[i].frame.data[5];
            double torqueNm = (static_cast<unsigned short>(torqueRaw) - 2047.0) / 2047.0 * 48.0;
            unsigned char id = received[i].frame.data[0] & 0xFF;
            // 获取最新的电机反馈值
            _mf.id = static_cast<int>(id);
            _mf.position = position;
            _mf.velocity = velocity;
            _mf.torque = torqueNm;
            // std::cout << "positionRaw:" << static_cast<unsigned short>(positionRaw) << std::endl;
            // std::cout << "velocityRaw:" << velocityRaw << std::endl;
            // std::cout << "torqueRaw:" << torqueRaw << std::endl;
            std::cout << "CanIndex: " << CanIndex << std::endl;
            std::cout << "电机ID: " << static_cast<int>(id) << ", 位置: " << position << ", 速度: " << velocity << ", 转矩: " << torqueNm << std::endl;

    } 
    }
    else if (receiveCount == -1) {
        std::cerr << "接收CAN消息出错" << std::endl;
    }
}    

void MotorController::keeper(){
    ZCAN_Receive_Data received[10];

}

//用于创建MotorDATA实例
MotorDATA *MotorDATACreate(){
    MotorDATA *motor_data = (MotorDATA*)malloc(sizeof(MotorDATA));
    motor_data->error_ = kMotorNoError;
    return motor_data;
}

//用于销毁MotorDATA实例
void MotorDATADestroy(MotorDATA *motor_data){
    free(motor_data);
}


//用于将MotorCMD中的float数据转换为CAN协议中发送的uint数据
void MotorController::FloatsToUints(const MotorCMD *param, uint8_t *data)
{
    uint16_t _position = FloatToUint(param->position_, POSITION_MIN, POSITION_MAX, SEND_POSITION_LENGTH);
    uint16_t _velocity = FloatToUint(param->velocity_, VELOCITY_MIN, VELOCITY_MAX, SEND_VELOCITY_LENGTH);
    uint16_t _torque = FloatToUint(param->torque_, TORQUE_MIN, TORQUE_MAX, SEND_TORQUE_LENGTH);
    uint16_t _kp = FloatToUint(param->kp_, KP_MIN, KP_MAX, SEND_KP_LENGTH);
    uint16_t _kd = FloatToUint(param->kd_, KD_MIN, KD_MAX, SEND_KD_LENGTH);
    data[0] = _position;
    data[1] = _position >> 8;
    data[2] = _velocity;
    data[3] = ((_velocity >> 8) & 0x3f)| ((_kp & 0x03) << 6);
    data[4] = _kp >> 2;
    data[5] = _kd;
    data[6] = _torque;
    data[7] = _torque >> 8;
}

//用于将CAN协议中收到的uint数据转换为MotorDATA中的float数据
void MotorController::UintsToFloats(const struct can_frame *frame, MotorDATA *data)
{
    const ReceivedMotionData *pcan_data = (const ReceivedMotionData*)frame->data;
    data->position_ = UintToFloat(pcan_data->position, POSITION_MIN, POSITION_MAX, RECEIVE_POSITION_LENGTH);
    data->velocity_ = UintToFloat(pcan_data->velocity, VELOCITY_MIN, VELOCITY_MAX, RECEIVE_VELOCITY_LENGTH);
    data->torque_ = UintToFloat(pcan_data->torque, TORQUE_MIN, TORQUE_MAX, RECEIVE_TORQUE_LENGTH);
    data->flag_ = (bool)pcan_data->temp_flag;
    if(data->flag_ == kMotorTempFlag){
        data->temp_ = UintToFloat(pcan_data->temperature, MOTOR_TEMP_MIN, MOTOR_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
    else{
        data->temp_ = UintToFloat(pcan_data->temperature, DRIVER_TEMP_MIN, DRIVER_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
}

//用于向云深处j60电机发送力矩环控制
void MotorController::sendTorqueToDeepMotor(unsigned char id, float set_torque)
{
    MotorCMD cmd;
    unsigned char data[8];

    cmd.torque_ = set_torque;
    cmd.velocity_ = 0;
    cmd.position_ = 0;
    cmd.kd_ = 0;
    cmd.kp_ = 0;
    cmd.cmd_ = 4;
    FloatsToUints(&cmd, data);

    can_frame canObj;
    canObj.can_id = (cmd.cmd_ << 5) | id;
    canObj.can_dlc = 8;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);

    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;
    // std::cout << "can: " << canIndex << " id: " << int(id) <<  " torque: " << torque << std::endl;

    // 发送数据包
    int result = ZCAN_Transmit(channel_handle, &Transmit_Data, 1);
    if (result == -1) {
        std::cerr << "发送数据失败，电机ID：" << static_cast<int>(id) << std::endl;
    } 
}


//用于向云深处j60电机发送位置环控制
void MotorController::sendPositionToDeepMotor(unsigned char id, float set_position, float kp, float kd)
{
    MotorCMD cmd;
    unsigned char data[8];

    cmd.torque_ = 0;
    cmd.velocity_ = 0;
    cmd.position_ = set_position;
    cmd.kd_ = kd;
    cmd.kp_ = kp;
    cmd.cmd_ = 4;
    FloatsToUints(&cmd, data);

    can_frame canObj;
    canObj.can_id = (cmd.cmd_ << 5) | id;
    canObj.can_dlc = 8;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);

    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;
    // std::cout << "can: " << canIndex << " id: " << int(id) <<  " torque: " << torque << std::endl;

    // 发送数据包
    int result = ZCAN_Transmit(channel_handle, &Transmit_Data, 1);
    if (result == -1) {
        std::cerr << "发送数据失败，电机ID：" << static_cast<int>(id) << std::endl;
    } 
}


//用于向云深处j60电机发送速度环控制
void MotorController::sendVelocityToDeepMotor(unsigned char id, float set_velocity, float kd)
{
    MotorCMD cmd;
    unsigned char data[8];

    cmd.torque_ = 0;
    cmd.velocity_ = set_velocity;
    cmd.position_ = 0;
    cmd.kd_ = kd;
    cmd.kp_ = 0;
    cmd.cmd_ = 4;
    FloatsToUints(&cmd, data);

    can_frame canObj;
    canObj.can_id = (cmd.cmd_ << 5) | id;
    canObj.can_dlc = 8;
    // canObj.RemoteFlag = 0;
    // canObj.ExternFlag = 0;
    // canObj.DataLen = 8;
    std::copy(data, data + 8, canObj.data);

    ZCAN_Transmit_Data Transmit_Data;
    Transmit_Data.frame = canObj;
    Transmit_Data.transmit_type = 0;
    // std::cout << "can: " << canIndex << " id: " << int(id) <<  " torque: " << torque << std::endl;

    // 发送数据包
    int result = ZCAN_Transmit(channel_handle, &Transmit_Data, 1);
    if (result == -1) {
        std::cerr << "发送数据失败，电机ID：" << static_cast<int>(id) << std::endl;
    } 
}

//用于根据收到的can帧进行MotorDATA的填充
void MotorController::ParseRecvFrame(const struct can_frame *frame_ret, MotorDATA *data){
    uint32_t frame_id = frame_ret->can_id;
    uint32_t cmd = (frame_id >> CAN_ID_SHIFT_BITS) & 0x3f;
    uint32_t motor_id = frame_id & 0x0f;
    data->motor_id_ = motor_id;
    data->cmd_ = cmd;
    switch (cmd)
    {
    case ENABLE_MOTOR:
        printf("[INFO] Motor with id: %d enable success\r\n", (uint32_t)motor_id);
        break;

    case DISABLE_MOTOR:
        printf("[INFO] Motor with id: %d disable success\r\n", (uint32_t)motor_id);
        break;

    case SET_HOME:
        printf("[INFO] Motor with id: %d set zero point success\r\n", (uint32_t)motor_id);
        break;

    case ERROR_RESET:
        printf("[INFO] Motor with id: %d clear error success\r\n", (uint32_t)motor_id);
        break;

    case CONTROL_MOTOR:
        UintsToFloats(frame_ret, data);
        // printf("[INFO] Motor with id: %d position: %f velocity: %f torque: %f\r\n", (uint32_t)motor_id, data->position_, data->velocity_, data->torque_);
        pos = data->position_;
        veloc = data->velocity_;
        tor = data->torque_;
        break;    

    case GET_STATUS_WORD:
        data->error_ = (frame_ret->data[0] << 8) | frame_ret->data[1];
        break;

    default:
        printf("[WARN] Received a frame not fitting into any cmd\r\n");
        break;
    }
}



//接收云深处j60电机数据
void MotorController::receiveDataFromDeepMotor()
{
    ZCAN_Receive_Data received[1];
    int receiveCount = ZCAN_Receive(channel_handle, received, 1, 1);

    if (receiveCount > 0) {
        for (int i = 0; i < receiveCount; ++i) {
            can_frame frame = received[i].frame;
            MotorDATA rec;
            ParseRecvFrame(&frame, &rec);
            std::cout << "ID: " << static_cast<int>(rec.motor_id_) << " Position: " << rec.position_ << " Velocity: " << rec.velocity_ << " Torque: " << rec.torque_ << std::endl;
            _mf.id = rec.motor_id_;
            _mf.position = rec.position_;
            _mf.velocity = rec.velocity_;
            _mf.torque = rec.torque_;
        }
    }

}
