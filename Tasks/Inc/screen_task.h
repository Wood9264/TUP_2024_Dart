#ifndef _SCREEN_TASK_H_
#define _SCREEN_TASK_H_

#include "tjc_usart_hmi.h"

#define SCREEN_FRAME_HEADER 0xA5
#define SCREEN_FRAME_TAIL 0xFF

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus

    class screen_t
    {
    public:

        screen_t();
        void send_data();
        void data_analysis();
    };

#endif

    extern void screen_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif
#endif
