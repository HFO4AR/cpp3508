//
// Created by nyuki on 2025/10/17.
//

#include "motorTask.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "Dji_Rm3508.h"
#include "DM_S3519.h"

DM_S3519 motor4(1);
extern "C" void motor_task(void const * argument){

    while (1) {

        // motor0.set_pos(100);
        motor4.set_spd(6.0f);
        // Dji_Rm3508::send_data();
        osDelay(10);
    }
}