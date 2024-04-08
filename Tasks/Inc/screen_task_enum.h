#ifndef _SCREEN_TASK_ENUM_H_
#define _SCREEN_TASK_ENUM_H_

//命令ID
enum
{
    ID_outpost_speed_1 = 7,
    ID_outpost_speed_2 = 8,
    ID_outpost_speed_3 = 9,
    ID_outpost_speed_4 = 10,

    ID_base_speed_1 = 11,
    ID_base_speed_2 = 12,
    ID_base_speed_3 = 13,
    ID_base_speed_4 = 14,

    ID_outpost_yaw_offset_num_1 = 15,
    ID_outpost_yaw_offset_num_2 = 16,
    ID_outpost_yaw_offset_num_3 = 17,
    ID_outpost_yaw_offset_num_4 = 18,

    ID_base_yaw_offset_num_1 = 19,
    ID_base_yaw_offset_num_2 = 20,
    ID_base_yaw_offset_num_3 = 21,
    ID_base_yaw_offset_num_4 = 22,

    ID_refresh = 100,
    ID_fric_monitor = 101,
    ID_other_monitor = 102,
    ID_main_page_message = 103,
    ID_warning_message = 104,
} cmd_ID_e;

#endif
