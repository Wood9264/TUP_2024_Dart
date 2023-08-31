#include "OLED_task.h"
#include "OLED.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

void OLED_task(void const *pvParameters)
{
    OLED_init();
    vTaskDelay(1000);

    while (1)
    {
        OLED_show_string(0, 0, "Hello World!");
        OLED_refresh_gram();
        vTaskDelay(2);
    }
}
