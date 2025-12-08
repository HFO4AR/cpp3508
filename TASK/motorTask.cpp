//
// Created by nyuki on 2025/10/17.
//

#include "motorTask.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "Dji_Rm3508.h"
#include "DM_S3519.h"

DM_S3519 motor4(1);
extern Dji_Rm3508 motor0;
extern "C" void motor_task(void const * argument){
    motor0.spd_pid_init(1,0,0.1);
    motor0.set_spd_max_output(1500);
    motor0.set_cur_ol(0);
    while (1) {

        motor0.set_spd(1000);
        // motor4.set_spd(6.0f);
        // Dji_Rm3508::send_data();
        osDelay(10);
    }
}