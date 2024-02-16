#ifndef _SCREEN_TASK_H_
#define _SCREEN_TASK_H_

#include "tjc_usart_hmi.h"

#define SCREEN_FRAME_HEADER 0xA5 //帧头0xA5
#define SCREEN_FRAME_TAIL 0xFF   //帧尾连续三个0xFF
#define CMD_ID_LENTH 1           //命令ID长度
#define MIN_FRAME_LENGTH 6       //最小帧长度
#define MAX_FRAME_LENGTH 11      //最大帧长度

#define SCREEN_TASK_INIT_TIME 3000 //屏幕任务初始化时间

#define INIT_OUTPOST_SPEED 2000       //初始化前哨站速度
#define INIT_BASE_SPEED 4000          //初始化基地速度
#define INIT_OUTPOST_YAW_OFFSET_NUM 0 //初始化前哨站YAW补偿量
#define INIT_BASE_YAW_OFFSET_NUM 0    //初始化基地YAW补偿量

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus

    class screen_t
    {
    public:
        int16_t outpost_speed[4];
        int16_t base_speed[4];
        fp32 outpost_yaw_offset_num[4];
        fp32 base_yaw_offset_num[4];

        screen_t();
        void screen_data_init();
        void send_data();
        void data_analysis();
        void frame_content_analysis(uint8_t *frame_content);
        int16_t ascii_to_int16_t(const uint8_t *str);
        fp32 ascii_to_fp32(const uint8_t *str);
    };

    extern screen_t *screen_point(void);

#endif

    extern void screen_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif
#endif
