#include "OLED_task.h"
#include "OLED.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

extern revolver_task_t revolver;

void OLED_task(void const *pvParameters)
{
    OLED_init();
    vTaskDelay(1000);

    while (1)
    {
        OLED_operate_gram(PEN_CLEAR);

        OLED_printf(1, 8, "%.0f", revolver.fric_speed_offset + BASE_SPEED);

        OLED_printf(0, 16, "%d", revolver.fric_motor[0].motor_measure->speed_rpm);
        OLED_printf(0, 0, "%d", revolver.fric_motor[1].motor_measure->speed_rpm);
        OLED_printf(4, 16, "%d", revolver.fric_motor[2].motor_measure->speed_rpm);
        OLED_printf(4, 0, "%d", revolver.fric_motor[3].motor_measure->speed_rpm);

        OLED_printf(2, 14, "%d", revolver.fric_motor[0].motor_measure->temperate);
        OLED_printf(2, 5, "%d", revolver.fric_motor[1].motor_measure->temperate);
        OLED_printf(3, 14, "%d", revolver.fric_motor[2].motor_measure->temperate);
        OLED_printf(3, 5, "%d", revolver.fric_motor[3].motor_measure->temperate);

        OLED_refresh_gram();
        vTaskDelay(2);
    }
}
