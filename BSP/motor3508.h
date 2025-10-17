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

class motor3508 {
public:
    int motor_enable=MOTOR_DISABLE;
    const int id;
    int16_t pos=0;
    int16_t spd=0;
    int16_t cur=0;
    int8_t temp=0;

    int32_t total_pos=0;
    int16_t last_pos=0;
    int16_t round=0;

    typedef struct {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        int last_pos = 0;
        int Kp_output = 0;
        int Ki_output = 0;
        int Kd_output = 0;
        int output = 0;
        int deadband = 0;
        int integral=0;
        int error=0;
        int last_error=0;
        int target=0;
        int actual=0;
        int max_output = 0;
    } pid_data_t;

    pid_data_t pos_pid_data;
    pid_data_t spd_pid_data;

    //init
    motor3508(const int id) : id(id) {
        motor_enable=MOTOR_ENABLE;

    }

    void pos_pid_init(float kp, float ki, float kd);

    void spd_pid_init(float kp, float ki, float kd);

    void set_cur(int target);

    void set_spd(int target);

    void set_single_pos(int target);

    void set_pos(int target);

private:

    void total_pos_updata();

    void pid_compuate(pid_data_t *pid_data);
};

extern motor3508 motor0;
extern motor3508 motor1;
extern motor3508 motor2;
extern motor3508 motor3;

// void Motor_Set_Current(int16_t current0, int16_t current1, int16_t current2, int16_t current3);
#endif

#endif //CPP3508_MOTOR3508_H