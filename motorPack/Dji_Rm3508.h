//
// Created by nyuki on 2025/10/17.
//

#ifndef CPP3508_MOTOR3508_H
#define CPP3508_MOTOR3508_H

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

    void get_rm3508_data();

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

# define DISABLE 0
# define ENABLE 1
class Dji_Rm3508 :public Motor{
public:
    Dji_Rm3508(const int id) :  Motor(id){}
    static void send_data();
    void set_cur_ol(int target);//open loop
    void enable_sync_seed();
private:
    int sync_seed_mode=DISABLE;
};



// void Motor_Set_Current(int16_t current0, int16_t current1, int16_t current2, int16_t current3);
#endif

#endif //CPP3508_MOTOR3508_H