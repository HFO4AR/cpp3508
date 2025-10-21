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
#include "motor3508.h"

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "cmath"
#include "iostream"
#include "map"
#include <array>
#include <algorithm>
#include "cstdlib"
using namespace std;
motor3508 motor0(0);
motor3508 motor1(1);
motor3508 motor2(2);
motor3508 motor3(3);


motor3508* motor3508_index[4]={&motor0, &motor1, &motor2, &motor3};


//can接收函数
union rx_data_t{
    array<uint8_t,8> input;
    struct{
        int8_t null_data=0;
        int8_t temp=0;
        int16_t cur=0;
        int16_t spd=0;
        int16_t pos=0;
    }read;
};

extern "C" void get_3508_data() {
    rx_data_t RxData{};
    CAN_RxHeaderTypeDef RxHeader;
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData.input.data());
    reverse(RxData.input.begin(), RxData.input.end());
    int motor_id=RxHeader.StdId-0x200-1;
    motor3508_index[motor_id]->pos=RxData.read.pos;
    motor3508_index[motor_id]->cur=RxData.read.cur;
    motor3508_index[motor_id]->spd=RxData.read.spd;
    motor3508_index[motor_id]->temp=RxData.read.temp;
    if (RxData.read.temp) {
        motor3508_index[motor_id]->motor_enable=MOTOR_ENABLE;
    }

}

//can发送函数
uint8_t TxData[8];

void send_3508_data() {
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef tx;
    tx.StdId = 0x200;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan2, &tx, TxData, &TxMailbox);
    fill_n(TxData, 8, 0); //清空发送缓存
}



/****motor3508类实现****/
void motor3508::pos_pid_init(float kp, float ki, float kd) {
    pos_pid_data.Kp = kp;
    pos_pid_data.Ki = ki;
    pos_pid_data.Kd = kd;
    pos_pid_data.max_output = 1000;
}

void motor3508::spd_pid_init(float kp, float ki, float kd) {
    spd_pid_data.Kp = kp;
    spd_pid_data.Ki = ki;
    spd_pid_data.Kd = kd;
    pos_pid_data.max_output = 1000;
}



void motor3508::set_cur(int target) {
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
    total_pos_updata();
}


void motor3508::set_spd(int target) {
    spd_pid_data.actual=spd;
    spd_pid_data.target = target;
    pid_compuate(&spd_pid_data);
    set_cur_cl(spd_pid_data.output);
}

void motor3508::set_single_pos(int target) {
    pos_pid_data.actual=pos;
    pos_pid_data.target = target;
    pid_compuate(&pos_pid_data);
    set_spd(pos_pid_data.output);
}

void motor3508::set_pos(int target) {
    pos_pid_data.actual=total_pos;
    pos_pid_data.target = target;
    pid_compuate(&pos_pid_data);
    set_spd(pos_pid_data.output);
}

void motor3508::pid_compuate(pid_data_t *pid_data) {
    pid_data->error = pid_data->target - pid_data->actual;
    pid_data->Kp_output = pid_data->Kp * pid_data->error;
    pid_data->integral += pid_data->error;
    pid_data->Ki_output += pid_data->Ki * pid_data->integral;
    pid_data->Kd_output = pid_data->Kd * (pid_data->error - pid_data->last_error);

    if (pid_data->output > pid_data->max_output) {
        pid_data->output = pid_data->max_output;
        pid_data->integral = pid_data->integral - pid_data->error;
    } else if (pid_data->output < -pid_data->max_output) {
        pid_data->output = -pid_data->max_output;
        pid_data->integral = pid_data->integral - pid_data->error;
    }

    if (abs(pid_data->error) < abs(pid_data->deadband)) {
        pid_data->output = 0;
    }

    pid_data->output = pid_data->Kp_output + pid_data->Ki_output + pid_data->Kd_output;
}

void motor3508::total_pos_updata() {
    if (pos - last_pos > 4096) {
        round--;
    } else if (pos-last_pos < -4096) {
        round++;
    }
    last_pos = pos;
    total_pos = round * 8192 + pos;
}

void motor3508::set_spd_max_output(int val) {
    spd_pid_data.max_output = val;
}

void motor3508::set_pos_max_output(int val) {
    pos_pid_data.max_output = val;
}

void motor3508::set_spd_deadband(int val) {
    spd_pid_data.deadband = val;
}


void motor3508::set_pos_deadband(int val) {
    pos_pid_data.deadband = val;
}

void motor3508::set_cur_cl(int target) {
    if (motor_enable) {
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
        motor_enable = MOTOR_DISABLE;
    }
    total_pos_updata();
}



// uint32_t TxMailbox;
// void Motor_Set_Current(int16_t current0, int16_t current1, int16_t current2, int16_t current3) {
//
//     CAN_TxHeaderTypeDef tx = {
//         .StdId = 0x200, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8
//     };
//     uint8_t TxData[8] = {0};
//
//     TxData[0] = (current0 >> 8) & 0xFF;
//     TxData[1] = current0 & 0xFF;
//     TxData[2] = (current1 >> 8) & 0xFF;
//     TxData[3] = current1 & 0xFF;
//     TxData[4] = (current2 >> 8) & 0xFF;
//     TxData[5] = current2 & 0xFF;
//     TxData[6] = (current3 >> 8) & 0xFF;
//     TxData[7] = current3 & 0xFF;
//     HAL_CAN_AddTxMessage(&hcan2, &tx, TxData, &TxMailbox);
// }