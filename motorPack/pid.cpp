//
// Created by nyuki on 2025/11/2.
//

#include "pid.h"
#include "cmath"
void Pid::init(float kp, float ki, float kd, float kaw, int max_output) {
    data.Kaw = kaw;
    data.Kp = kp;
    data.Ki = ki;
    data.Kd = kd;
    data.max_output = max_output;
}
void Pid::compuate() {
    data.error = data.target - data.actual;
    data.Kp_output = data.Kp * data.error;
    data.Kd_output = data.Kd * (data.error - data.last_error);
    data.Ki_output += data.Ki * data.error;

    //计算未饱和输出（理论输出）
    float unsat_output = data.Kp_output + data.Kd_output + data.Ki_output;
    float sat_output = unsat_output;
    //输出限幅
    if (unsat_output > data.max_output) {
        sat_output = data.max_output;
    } else if (unsat_output < -data.max_output) {
        sat_output = -data.max_output;
    }
    data.Ki_output += data.Kaw * (sat_output - unsat_output);

    data.output = sat_output;

    if (abs(data.error) < abs(data.deadband)) {
        data.output = 0;
    }
    data.last_error = data.error;
}