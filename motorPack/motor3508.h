//
// Created by nyuki on 2025/10/17.
//

#ifndef CPP3508_MOTOR3508_H
#define CPP3508_MOTOR3508_H
#include <sys/_stdint.h>
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

    void get_3508_data();

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

void send_3508_data();

class motor3508 :public Motor{
public:
    motor3508(const int id) :  Motor(id){}
    void set_cur(int target);//open loop
protected:
    void set_cur_cl(int target);

    void total_pos_updata();

};



// void Motor_Set_Current(int16_t current0, int16_t current1, int16_t current2, int16_t current3);
#endif

#endif //CPP3508_MOTOR3508_H