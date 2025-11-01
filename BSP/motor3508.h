//
// Created by nyuki on 2025/10/17.
//

#ifndef CPP3508_MOTOR3508_H
#define CPP3508_MOTOR3508_H
#include <sys/_stdint.h>



#ifdef __cplusplus
extern "C" {
#endif

    void get_3508_data();

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define MOTOR_ENABLE 1
#define MOTOR_DISABLE 0

void send_3508_data();
class motor {
public:
    motor (const int id):id(id) {
    }
    int16_t pos=0;
    int16_t spd=0;
    int16_t cur=0;
    int8_t temp=0;
    int motor_enable=MOTOR_DISABLE;
    virtual void set_cur(int target);//open loop

    virtual void set_spd(int target);

    virtual void set_single_pos(int target);

    virtual void set_pos(int target);

    virtual void set_spd_max_output(int val);

    virtual void set_pos_max_output(int val);

    virtual void set_spd_deadband(int val);

    virtual void set_pos_deadband(int val);
protected:
    const int id;
    int32_t total_pos=0;
    int16_t last_pos=0;
    int16_t round=0;
};

class pid {
protected:
    pid () = default;
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

    pid_data_t pos_pid_data;
    pid_data_t spd_pid_data;
public:
    void pos_pid_init(float kp, float ki, float kd) {
        pos_pid_init(kp,ki,kd,kp/ki);
    }
    void pos_pid_init(float kp, float ki, float kd, float kaw,int max_output=1000);

    void spd_pid_init(float kp, float ki, float kd) {
        spd_pid_init(kp,ki,kd,kp/ki);
    }
    void spd_pid_init(float kp, float ki, float kd, float kaw,int max_output=1000);

    void set_Kaw(int val);
};



class motor3508 :public pid,public motor{
public:
    motor3508(const int id) : pid(), motor(id) {
        // motor_enable=MOTOR_ENABLE;
    }

    void set_cur(int target);//open loop

    void set_spd(int target);

    void set_single_pos(int target);

    void set_pos(int target);

    void set_spd_max_output(int val);

    void set_pos_max_output(int val);

    void set_spd_deadband(int val);

    void set_pos_deadband(int val);

protected:
    void total_pos_updata();

    void pid_compuate(pid_data_t *pid_data);

    void set_cur_cl(int target);//close loop
};

extern motor3508 motor0;
extern motor3508 motor1;
extern motor3508 motor2;
extern motor3508 motor3;

// void Motor_Set_Current(int16_t current0, int16_t current1, int16_t current2, int16_t current3);
#endif

#endif //CPP3508_MOTOR3508_H