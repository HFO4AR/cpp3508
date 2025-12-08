//
// Created by nyuki on 2025/10/17.
//
/***
 *使用说明：
 *调用set_cur方法设置电机电流
 *调用set_pos方法设置电机位置
 *调用set_spd方法设置电机速度
 *
 *调用set_spd和set_pos方法之前必须先调用pos_pid_init和spd_pid_init方法进行PID参数初始化，否则PID输出为0
 *
 *调用完set_cur、set_pos、set_spd方法之后必须调用send_3508_data方法发送数据
 *
 *
 ***/
#include "Dji_Rm3508.h"

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "cmath"
#include "iostream"
#include "map"
#include <array>
#include <algorithm>
#include "cstdlib"
using namespace std;
Dji_Rm3508 motor0(0);
Dji_Rm3508 motor1(1);
Dji_Rm3508 motor2(2);
Dji_Rm3508 motor3(3);


Dji_Rm3508 *motor3508_index[4] = {&motor0, &motor1, &motor2, &motor3};

//can接收函数


extern "C" void get_rm3508_data() {
    union rx_data_t {
        array<uint8_t, 8> input;

        struct {
            int8_t null_data = 0;
            int8_t temp = 0;
            int16_t cur = 0;
            int16_t spd = 0;
            int16_t pos = 0;
        } read;
    };
    rx_data_t RxData{};

    CAN_RxHeaderTypeDef RxHeader;
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData.input.data());
    reverse(RxData.input.begin(), RxData.input.end());
    int motor_id = RxHeader.StdId - 0x200 - 1;
    motor3508_index[motor_id]->pos = RxData.read.pos;
    motor3508_index[motor_id]->cur = RxData.read.cur;
    motor3508_index[motor_id]->spd = RxData.read.spd;
    motor3508_index[motor_id]->temp = RxData.read.temp;
    if (RxData.read.temp) {
        motor3508_index[motor_id]->motor_enable = MOTOR_ENABLE;
    }
}

//can发送函数

uint8_t TxData[8];
void Dji_Rm3508::send_data() {
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef tx;
    tx.StdId = 0x200;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan2, &tx, TxData, &TxMailbox);
    fill_n(TxData, sizeof(TxData), 0); //清空发送缓存
}



void Dji_Rm3508::set_cur_ol(int target) {
    switch (id) {
        case 0:
            TxData[0] = (target >> 8) & 0xFF;
            TxData[1] = target & 0xFF;
            break;
        case 1:
            TxData[2] = (target >> 8) & 0xFF;
            TxData[3] = target & 0xFF;
            break;
        case 2:
            TxData[4] = (target >> 8) & 0xFF;
            TxData[5] = target & 0xFF;
            break;
        case 3:
            TxData[6] = (target >> 8) & 0xFF;
            TxData[7] = target & 0xFF;
            break;
        default:
            break;
    }
    if (sync_seed_mode!=ENABLE) {
        send_data();
    }
    total_pos_updata();
}


void Dji_Rm3508::enable_sync_seed() {
    sync_seed_mode=ENABLE;
}