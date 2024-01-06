#include "screen_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "tjc_usart_hmi.h"
#include "revolver_task.h"

void screen_task(void const *pvParameters)
{
    vTaskDelay(2000);

    while (1)
    {
        //发送电机转速
        TJCPrintf("t5.txt=\"%d\"", revolver_point()->fric_motor[0].motor_measure->speed_rpm);
        TJCPrintf("t6.txt=\"%d\"", revolver_point()->fric_motor[1].motor_measure->speed_rpm);
        TJCPrintf("t7.txt=\"%d\"", revolver_point()->fric_motor[2].motor_measure->speed_rpm);
        TJCPrintf("t8.txt=\"%d\"", revolver_point()->fric_motor[3].motor_measure->speed_rpm);

        //发送电机温度
        TJCPrintf("t9.txt=\"%d\"", revolver_point()->fric_motor[0].motor_measure->temperate);
        TJCPrintf("t10.txt=\"%d\"", revolver_point()->fric_motor[1].motor_measure->temperate);
        TJCPrintf("t11.txt=\"%d\"", revolver_point()->fric_motor[2].motor_measure->temperate);
        TJCPrintf("t12.txt=\"%d\"", revolver_point()->fric_motor[3].motor_measure->temperate);

        //发送转速设定值
        TJCPrintf("t13.txt=\"%d\"", BASE_SPEED + revolver_point()->outpost_speed_offset[0]);
        TJCPrintf("t14.txt=\"%d\"", BASE_SPEED + revolver_point()->outpost_speed_offset[1]);
        TJCPrintf("t15.txt=\"%d\"", BASE_SPEED + revolver_point()->outpost_speed_offset[2]);
        TJCPrintf("t16.txt=\"%d\"", BASE_SPEED + revolver_point()->outpost_speed_offset[3]);

        //发送转速补偿
        TJCPrintf("t17.txt=\"%d\"", revolver_point()->outpost_speed_offset[0]);
        TJCPrintf("t18.txt=\"%d\"", revolver_point()->outpost_speed_offset[1]);
        TJCPrintf("t19.txt=\"%d\"", revolver_point()->outpost_speed_offset[2]);
        TJCPrintf("t20.txt=\"%d\"", revolver_point()->outpost_speed_offset[3]);

        // yaw轴补偿
        TJCPrintf("t25.txt=\"%.2f\"", revolver_point()->yaw_motor.offset_num[0]);
        TJCPrintf("t26.txt=\"%.2f\"", revolver_point()->yaw_motor.offset_num[1]);
        TJCPrintf("t27.txt=\"%.2f\"", revolver_point()->yaw_motor.offset_num[2]);
        TJCPrintf("t28.txt=\"%.2f\"", revolver_point()->yaw_motor.offset_num[3]);

        vTaskDelay(2);
    }
}
