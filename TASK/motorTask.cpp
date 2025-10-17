//
// Created by nyuki on 2025/10/17.
//

#include "motorTask.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "motor3508.h"

extern "C" void motor_task(void const * argument){
    motor0.spd_pid_init(9.0,0,0);
    while (1) {
        motor0.set_spd(100);
        send_3508_data();
        osDelay(10);
    }
}