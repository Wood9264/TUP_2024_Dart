/**
 * @file OLED_task.cpp
 * @author Yang Maolin (1831051389@qq.com)
 * @brief OLED显示任务相关代码。原用于OLED显示电机速度、温度等信息，因为使用了串口屏，现已废弃。
 */
#include "OLED_task.h"
#include "OLED.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

static void GENSHIN_START();

extern revolver_task_t revolver;

void OLED_task(void const *pvParameters)
{
    vTaskDelay(2000);
    OLED_init();
    //原神，启动！
    GENSHIN_START();

    while (1)
    {
        OLED_operate_gram(PEN_CLEAR);

        // OLED_printf(1, 8, "%.0f", revolver.fric_speed_offset + BASE_SPEED);

        OLED_printf(0, 0, "%d", -revolver.fric_wheel_group.fric_motor[0].motor_measure->speed_rpm);
        OLED_printf(0, 16, "%d", revolver.fric_wheel_group.fric_motor[1].motor_measure->speed_rpm);
        OLED_printf(4, 0, "%d", -revolver.fric_wheel_group.fric_motor[2].motor_measure->speed_rpm);
        OLED_printf(4, 16, "%d", revolver.fric_wheel_group.fric_motor[3].motor_measure->speed_rpm);

        OLED_printf(2, 5, "%d", revolver.fric_wheel_group.fric_motor[0].motor_measure->temperate);
        OLED_printf(2, 14, "%d", revolver.fric_wheel_group.fric_motor[1].motor_measure->temperate);
        OLED_printf(3, 5, "%d", revolver.fric_wheel_group.fric_motor[2].motor_measure->temperate);
        OLED_printf(3, 14, "%d", revolver.fric_wheel_group.fric_motor[3].motor_measure->temperate);

        OLED_refresh_gram();
        vTaskDelay(2);
    }
}

static void GENSHIN_START()
{
    oled_write_byte(0x81, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);

    OLED_GENSHIN_LOGO();
    for (int i = 0x0; i < 0x80; i++)
    {
        oled_write_byte(0x81, OLED_CMD);
        oled_write_byte(i, OLED_CMD);
        vTaskDelay(10);
    }
    for (int i = 0x81; i < 0xff; i++)
    {
        oled_write_byte(0x81, OLED_CMD);
        oled_write_byte(i, OLED_CMD);
        vTaskDelay(2);
    }
}
