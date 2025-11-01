//
// Created by nyuki on 2025/11/2.
//

#ifndef CPP3508_PID_H
#define CPP3508_PID_H
#ifdef __cplusplus
class Pid {
public:
    typedef struct {
        float Kp;
        float Ki;
        float Kd;
        float Kaw;//抗饱和反馈系数（建议：Kaw ≈ Ki/Kp 或 Ki 的 0.5~1倍）
        int last_pos;
        int Kp_output;
        int Ki_output;
        int Kd_output;
        int output;
        int deadband;
        int error;
        int last_error;
        int target;
        int actual;
        int max_output;
    } pid_data_t;

    pid_data_t data;

    Pid () =default;
    void compuate();
    void init(float kp, float ki, float kd ,float kaw, int max_output);


};
#endif

#endif //CPP3508_PID_H