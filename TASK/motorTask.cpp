//
// Created by nyuki on 2025/10/17.
//

#include "motorTask.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "motor3508.h"

extern "C" void motor_task(void const * argument){
    motor0.spd_pid_init(9.0,0,0);
    motor0.pos_pid_init(0.5,0,0);
    while (1) {
        // motor0.set_spd(1000);
        motor0.pos_pid_data.max_output=2000;
        motor0.pos_pid_data.min_output=-2000;
        motor0.spd_pid_data.max_output=20000;
        motor0.spd_pid_data.min_output=-20000;
        motor0.set_pos(100);
        send_3508_data();
        osDelay(1);
    }
}