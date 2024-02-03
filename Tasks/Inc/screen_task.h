#ifndef _SCREEN_TASK_H_
#define _SCREEN_TASK_H_

#include "tjc_usart_hmi.h"

#define SCREEN_FRAME_HEADER 0xA5 //帧头0xA5
#define SCREEN_FRAME_TAIL 0xFF   //帧尾连续三个0xFF
#define CMD_ID_LENTH 1           //命令ID长度
#define MIN_FRAME_LENGTH 6       //最小帧长度
#define MAX_FRAME_LENGTH 11      //最大帧长度

#define SCREEN_TASK_INIT_TIME 3000 //屏幕任务初始化时间

#define YAW_INITIAL_OFFSET_NUM_1 30.3
#define YAW_INITIAL_OFFSET_NUM_2 70.5
#define YAW_INITIAL_OFFSET_NUM_3 -0.5
#define YAW_INITIAL_OFFSET_NUM_4 -30.3

#define SPEED_INITIAL_OFFSET_1 0
#define SPEED_INITIAL_OFFSET_2 500
#define SPEED_INITIAL_OFFSET_3 1000
#define SPEED_INITIAL_OFFSET_4 -500

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus

    class screen_t
    {
    public:
        int16_t speed_offset_1;
        int16_t speed_offset_2;
        int16_t speed_offset_3;
        int16_t speed_offset_4;
        fp32 yaw_offset_num_1;
        fp32 yaw_offset_num_2;
        fp32 yaw_offset_num_3;
        fp32 yaw_offset_num_4;

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
