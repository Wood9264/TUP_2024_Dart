/**
 * @file screen_task.cpp
 * @author Yang Maolin (1831051389@qq.com)
 * @brief 串口屏任务相关代码。包含串口屏数据的解析和交互。
 */
#include "screen_task.h"
#include "screen_task_enum.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "tjc_usart_hmi.h"
#include "revolver_task.h"
#include "usart.h"
#include "load_task.h"
#include "CRC8_CRC16.h"

screen_t screen;

screen_t *screen_point(void)
{
    return &screen;
}

/**
 * @brief 串口屏任务
 * @param pvParameters
 */
void screen_task(void const *pvParameters)
{
    vTaskDelay(SCREEN_TASK_INIT_TIME);
    //串口屏初始化
    screen.screen_init();

    while (1)
    {
        //串口屏数据解析
        screen.data_analysis();

        vTaskDelay(2);
    }
}

/**
 * @brief   串口屏构造函数
 */
screen_t::screen_t()
{
    initRingBuff();

    for (uint8_t i = 0; i < 4; i++)
    {
        outpost_speed[i] = INIT_OUTPOST_SPEED;
        base_speed[i] = INIT_BASE_SPEED;
        outpost_yaw_offset_num[i] = INIT_OUTPOST_YAW_OFFSET_NUM;
        base_yaw_offset_num[i] = INIT_BASE_YAW_OFFSET_NUM;
    }
}

/**
 * @brief   串口屏初始化
 * @note    需要先等待串口屏启动完成，所以不能使用构造函数来初始化
 */
void screen_t::screen_init()
{
    //清空屏幕串口缓冲区里的无关数据，消除上电过程中引脚杂波的影响
    uint8_t init_data[] = {0x00, 0xff, 0xff, 0xff};
    HAL_UART_Transmit(&huart1, init_data, 4, 100);
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET)
        ;
}

/**
 * @brief   串口屏数据解析
 */
void screen_t::data_analysis()
{
    //帧内容
    uint8_t frame_content[MAX_FRAME_LENGTH] = {0};
    //有效数据
    uint8_t vavid_data[MAX_FRAME_LENGTH] = {0};
    //单个字节数据
    uint8_t single_byte;
    //帧长度
    uint8_t frame_length = 0;

    //判断缓冲区中是否有足够的数据
    if (getRingBuffLenght() >= MIN_FRAME_LENGTH)
    {
        //遍历缓冲区寻找帧头并删除帧头之前的错误数据
        for (uint8_t i = 0; i < RINGBUFF_LEN; i++)
        {
            single_byte = read1BFromRingBuff(0);

            if (single_byte == SCREEN_FRAME_HEADER)
            {
                //读取最大长度的帧内容
                readNBFromRingBuff(frame_content, 0, MAX_FRAME_LENGTH);
                //统计帧长度，用于CRC16校验
                frame_length = SCREEN_FRAME_HEADER_LENTH + CMD_ID_LENTH + frame_content[1] + SCREEN_FRAME_TAIL_LENTH;
                //检查缓冲区的数据是否足够解析一帧
                if (getRingBuffLenght() >= frame_length)
                {
                    // CRC16校验
                    if (verify_CRC16_MODBUS_check_sum(frame_content, frame_length))
                    {
                        //切分有效数据
                        for (uint8_t j = 0; j < frame_content[1]; j++)
                        {
                            vavid_data[j] = frame_content[j + SCREEN_FRAME_HEADER_LENTH + CMD_ID_LENTH];
                        }
                        //解析有效数据
                        vavid_data_analysis(vavid_data, frame_content[SCREEN_FRAME_HEADER_LENTH]);
                        //删除已解析的帧
                        deleteRingBuff(frame_length);
                        //递归调用，继续解析下一帧
                        data_analysis();
                        return;
                    }
                    else
                    {
                        //如果CRC16校验失败，删除帧头
                        deleteRingBuff(1);
                    }
                }
                else
                {
                    return;
                }
            }
            else
            {
                deleteRingBuff(1);
            }
        }
    }
}

/**
 * @brief   有效数据解析
 * @param   vavid_data 有效数据
 * @param   cmd_ID 命令ID
 */
void screen_t::vavid_data_analysis(uint8_t *vavid_data, uint8_t cmd_ID)
{
    switch (cmd_ID)
    {
    case ID_outpost_speed_1:
        outpost_speed[0] = ascii_to_int16_t(vavid_data);
        break;
    case ID_outpost_speed_2:
        outpost_speed[1] = ascii_to_int16_t(vavid_data);
        break;
    case ID_outpost_speed_3:
        outpost_speed[2] = ascii_to_int16_t(vavid_data);
        break;
    case ID_outpost_speed_4:
        outpost_speed[3] = ascii_to_int16_t(vavid_data);
        break;
    case ID_base_speed_1:
        base_speed[0] = ascii_to_int16_t(vavid_data);
        break;
    case ID_base_speed_2:
        base_speed[1] = ascii_to_int16_t(vavid_data);
        break;
    case ID_base_speed_3:
        base_speed[2] = ascii_to_int16_t(vavid_data);
        break;
    case ID_base_speed_4:
        base_speed[3] = ascii_to_int16_t(vavid_data);
        break;

    case ID_outpost_yaw_offset_num_1:
        outpost_yaw_offset_num[0] = ascii_to_fp32(vavid_data);
        break;
    case ID_outpost_yaw_offset_num_2:
        outpost_yaw_offset_num[1] = ascii_to_fp32(vavid_data);
        break;
    case ID_outpost_yaw_offset_num_3:
        outpost_yaw_offset_num[2] = ascii_to_fp32(vavid_data);
        break;
    case ID_outpost_yaw_offset_num_4:
        outpost_yaw_offset_num[3] = ascii_to_fp32(vavid_data);
        break;
    case ID_base_yaw_offset_num_1:
        base_yaw_offset_num[0] = ascii_to_fp32(vavid_data);
        break;
    case ID_base_yaw_offset_num_2:
        base_yaw_offset_num[1] = ascii_to_fp32(vavid_data);
        break;
    case ID_base_yaw_offset_num_3:
        base_yaw_offset_num[2] = ascii_to_fp32(vavid_data);
        break;
    case ID_base_yaw_offset_num_4:
        base_yaw_offset_num[3] = ascii_to_fp32(vavid_data);
        break;

    case ID_refresh:
        refresh_data();
        break;
    case ID_fric_monitor:
        fric_monitor(vavid_data);
        break;
    case ID_other_monitor:
        other_monitor(vavid_data);
        break;
    case ID_main_page_message:
        main_page_message();
        break;
    case ID_warning_message:
        warning_message();
        break;
    }
}

/**
 * @brief   刷新串口屏数据
 */
void screen_t::refresh_data()
{
    TJCPrintf("t6.txt=\"%d\"", outpost_speed[0]);
    TJCPrintf("t7.txt=\"%d\"", outpost_speed[1]);
    TJCPrintf("t8.txt=\"%d\"", outpost_speed[2]);
    TJCPrintf("t9.txt=\"%d\"", outpost_speed[3]);
    TJCPrintf("t10.txt=\"%d\"", base_speed[0]);
    TJCPrintf("t11.txt=\"%d\"", base_speed[1]);
    TJCPrintf("t12.txt=\"%d\"", base_speed[2]);
    TJCPrintf("t13.txt=\"%d\"", base_speed[3]);
    TJCPrintf("t14.txt=\"%.2f\"", outpost_yaw_offset_num[0]);
    TJCPrintf("t15.txt=\"%.2f\"", outpost_yaw_offset_num[1]);
    TJCPrintf("t16.txt=\"%.2f\"", outpost_yaw_offset_num[2]);
    TJCPrintf("t17.txt=\"%.2f\"", outpost_yaw_offset_num[3]);
    TJCPrintf("t18.txt=\"%.2f\"", base_yaw_offset_num[0]);
    TJCPrintf("t19.txt=\"%.2f\"", base_yaw_offset_num[1]);
    TJCPrintf("t20.txt=\"%.2f\"", base_yaw_offset_num[2]);
    TJCPrintf("t21.txt=\"%.2f\"", base_yaw_offset_num[3]);
}

/**
 * @brief   摩擦轮数据监控
 * @param   selected_data 串口屏上复选框选择的数据，小端序
 * @note    串口屏上复选框选择的数据为两个字节小端序，每个字节的每一位代表一个复选框的选择状态
 *          例如：selected_data[0] = 0b00000000, selected_data[1] = 0b10000000，表示第一个复选框被选中，其他复选框未被选中
 *          阅读代码时请结合串口屏的工程文件来理解
 */
void screen_t::fric_monitor(uint8_t *selected_data)
{
    bool_t should_send_data[16] = {0};
    uint8_t i = 0;

    //一共16个复选框，遍历并获取每个复选框的选择状态
    for (i = 0; i < 8; i++)
    {
        should_send_data[i] = selected_data[1] >> 7 - i & 0x01;
        should_send_data[i + 8] = selected_data[0] >> 7 - i & 0x01;
    }

    for (i = 0; i < 16; i++)
    {
        if ((i == 0 || i == 4 || i == 8 || i == 12) && should_send_data[i])
        {
            TJCPrintf("t%d.txt=\"%d\"", i + 8, revolver_point()->fric_wheel_group.fric_motor[i / 4].motor_measure->speed_rpm);
        }
        else if ((i == 1 || i == 5 || i == 9 || i == 13) && should_send_data[i])
        {
            TJCPrintf("t%d.txt=\"%d\"", i + 8, revolver_point()->fric_wheel_group.fric_motor[(i - 1) / 4].motor_measure->temperate);
        }
        else if ((i == 2 || i == 6 || i == 10 || i == 14) && should_send_data[i])
        {
            TJCPrintf("t%d.txt=\"%d\"", i + 8, revolver_point()->fric_wheel_group.fric_motor[(i - 1) / 4].motor_measure->ecd);
        }
        else if ((i == 3 || i == 7 || i == 11 || i == 15) && should_send_data[i])
        {
            TJCPrintf("t%d.txt=\"%d\"", i + 8, revolver_point()->fric_wheel_group.fric_motor[(i - 1) / 4].motor_measure->given_current);
        }
    }
}

/**
 * @brief   其他数据监控
 * @param   options_data 串口屏上复选框选择的数据，小端序
 * @note    shoud_send_data[4]和shoud_send_data[9]未使用（即使选中也不发送数据）
 *          阅读代码时请结合串口屏的工程文件来理解
 */
void screen_t::other_monitor(uint8_t *options_data)
{
    bool_t should_send_data[16] = {0};
    uint8_t i = 0;

    //注意此处只有15个复选框，数据从options_data[1]的第二位开始
    for (i = 0; i < 8; i++)
    {
        if (i > 0)
        {
            should_send_data[i - 1] = options_data[1] >> 7 - i & 0x01;
        }
        should_send_data[i + 7] = options_data[0] >> 7 - i & 0x01;
    }

    if (should_send_data[0])
        TJCPrintf("t8.txt=\"%d\"", revolver_point()->yaw_motor.motor_measure->speed_rpm);
    if (should_send_data[1])
        TJCPrintf("t9.txt=\"%d\"", revolver_point()->yaw_motor.motor_measure->temperate);
    if (should_send_data[2])
        TJCPrintf("t10.txt=\"%d\"", revolver_point()->yaw_motor.motor_measure->ecd);
    if (should_send_data[3])
        TJCPrintf("t11.txt=\"%d\"", revolver_point()->yaw_motor.motor_measure->given_current);
    if (should_send_data[5])
        TJCPrintf("t13.txt=\"%d\"", load_point()->loader_motor.motor_measure->speed_rpm);
    if (should_send_data[6])
        TJCPrintf("t14.txt=\"%d\"", load_point()->loader_motor.motor_measure->temperate);
    if (should_send_data[7])
        TJCPrintf("t15.txt=\"%d\"", load_point()->loader_motor.motor_measure->ecd);
    if (should_send_data[8])
        TJCPrintf("t16.txt=\"%d\"", load_point()->loader_motor.motor_measure->given_current);
    if (should_send_data[10])
        TJCPrintf("t18.txt=\"%d\"", load_point()->rotary_motor.motor_measure->speed_rpm);
    if (should_send_data[11])
        TJCPrintf("t19.txt=\"%d\"", load_point()->rotary_motor.motor_measure->temperate);
    if (should_send_data[12])
        TJCPrintf("t20.txt=\"%d\"", load_point()->rotary_motor.motor_measure->ecd);
    if (should_send_data[13])
        TJCPrintf("t21.txt=\"%d\"", load_point()->rotary_motor.motor_measure->given_current);
    if (should_send_data[14])
        TJCPrintf("t22.txt=\"%.2f\"", load_point()->rotary_motor.relative_angle);
}

/**
 * @brief   主页信息
 * @note    因存在字符编码转换问题，主页信息使用英文字符
 */
void screen_t::main_page_message()
{
    TJCPrintf("t0.txt=\"Dart index: %d\"", syspoint()->active_dart_index);

    if (load_point()->loader_motor.has_shoot_init_finished && load_point()->rotary_motor.has_shoot_init_finished && revolver_point()->yaw_motor.has_shoot_init_finished)
    {
        TJCPrintf("t1.txt=\"Initialized\"");
        //字符颜色设为白色
        TJCPrintf("t1.pco=65535");
    }
    else
    {
        TJCPrintf("t1.txt=\"Uninitialized\"");
        //字符颜色设为红色
        TJCPrintf("t1.pco=63488");
    }

    if (syspoint()->strike_target == OUTPOST)
    {
        TJCPrintf("t2.txt=\"Target: outpost\"");
    }
    else
    {
        TJCPrintf("t2.txt=\"Target: base\"");
    }

    if (syspoint()->sys_mode == ZERO_FORCE)
    {
        TJCPrintf("t3.txt=\"ZERO FORCE\"");
    }
    else if (syspoint()->sys_mode == CALIBRATE)
    {
        TJCPrintf("t3.txt=\"CALIBRATE\"");
    }
    else if (syspoint()->sys_mode == SHOOT)
    {
        TJCPrintf("t3.txt=\"SHOOT\"");
    }

    TJCPrintf("t4.txt=\"Auto: OFF\"");
}

/**
 * @brief   警告信息
 */
void screen_t::warning_message()
{
    //装填电机未校准警告
    if (!load_point()->loader_motor.has_calibrated)
    {
        TJCPrintf("vis t0,1");
    }
    else
    {
        TJCPrintf("vis t0,0");
    }
    // yaw轴未校准警告
    if (!revolver_point()->yaw_motor.has_calibrated)
    {
        TJCPrintf("vis t1,1");
    }
    else
    {
        TJCPrintf("vis t1,0");
    }
    //装填电机受限警告
    if (load_point()->loader_motor.is_restricted_state)
    {
        TJCPrintf("vis t2,1");
    }
    else
    {
        TJCPrintf("vis t2,0");
    }
    //装填电机未初始化警告
    if (!load_point()->loader_motor.has_shoot_init_finished)
    {
        TJCPrintf("vis t3,1");
    }
    else
    {
        TJCPrintf("vis t3,0");
    }
    //转盘未初始化警告
    if (!load_point()->rotary_motor.has_shoot_init_finished)
    {
        TJCPrintf("vis t4,1");
    }
    else
    {
        TJCPrintf("vis t4,0");
    }
    // yaw轴未初始化警告
    if (!revolver_point()->yaw_motor.has_shoot_init_finished)
    {
        TJCPrintf("vis t5,1");
    }
    else
    {
        TJCPrintf("vis t5,0");
    }
    //遥控器离线警告
    if(toe_is_error(DBUS_TOE))
    {
        TJCPrintf("vis t6,1");
    }
    else
    {
        TJCPrintf("vis t6,0");
    }
    //摩擦轮电机1离线警告
    if(toe_is_error(FRIC_1_TOE))
    {
        TJCPrintf("vis t7,1");
    }
    else
    {
        TJCPrintf("vis t7,0");
    }
    //摩擦轮电机2离线警告
    if(toe_is_error(FRIC_2_TOE))
    {
        TJCPrintf("vis t8,1");
    }
    else
    {
        TJCPrintf("vis t8,0");
    }
    //摩擦轮电机3离线警告
    if(toe_is_error(FRIC_3_TOE))
    {
        TJCPrintf("vis t9,1");
    }
    else
    {
        TJCPrintf("vis t9,0");
    }
    //摩擦轮电机4离线警告
    if(toe_is_error(FRIC_4_TOE))
    {
        TJCPrintf("vis t10,1");
    }
    else
    {
        TJCPrintf("vis t10,0");
    }
    //yaw轴电机离线警告
    if(toe_is_error(YAW_MOTOR_TOE))
    {
        TJCPrintf("vis t11,1");
    }
    else
    {
        TJCPrintf("vis t11,0");
    }
    //装填电机离线警告
    if(toe_is_error(LOADER_MOTOR_TOE))
    {
        TJCPrintf("vis t12,1");
    }
    else
    {
        TJCPrintf("vis t12,0");
    }
    //转盘电机离线警告
    if(toe_is_error(ROTARY_MOTOR_TOE))
    {
        TJCPrintf("vis t13,1");
    }
    else
    {
        TJCPrintf("vis t13,0");
    }
}

/**
 * @brief   将字符串转换为整数
 * @param   str 字符串
 * @return  转换后的整数
 */
int16_t screen_t::ascii_to_int16_t(const uint8_t *str)
{
    int16_t result = 0;
    int8_t sign = 1;
    uint8_t i = 0;

    //检查数字是否为负数
    if (str[0] == '-')
    {
        sign = -1;
        i = 1;
    }

    //遍历字符串的每个字符
    while (str[i] != '\0')
    {
        if (str[i] >= '0' && str[i] <= '9')
        {
            result = result * 10 + (str[i] - '0');
        }
        else
        {
            return NULL;
        }
        i++;
    }

    result *= sign;

    return result;
}

/**
 * @brief   将字符串转换为浮点数
 * @param   str 字符串
 * @return  转换后的浮点数
 */
fp32 screen_t::ascii_to_fp32(const uint8_t *str)
{
    fp32 result = 0.0f;
    fp32 sign = 1.0f;
    uint8_t i = 0;

    //检查数字是否为负数
    if (str[0] == '-')
    {
        sign = -1.0f;
        i = 1;
    }

    //遍历字符串的每个字符
    while (str[i] != '\0')
    {
        if (str[i] >= '0' && str[i] <= '9')
        {
            result = result * 10.0f + (str[i] - '0');
        }
        else if (str[i] == '.')
        {
            //跳过小数点
            i++;
            fp32 decimal = 0.1f;
            //遍历小数点后的每个字符
            while (str[i] != '\0')
            {
                if (str[i] >= '0' && str[i] <= '9')
                {
                    result += (str[i] - '0') * decimal;
                    decimal *= 0.1f;
                }
                else
                {
                    return NULL;
                }
                i++;
            }
            break;
        }
        else
        {
            return NULL;
        }
        i++;
    }

    result *= sign;

    return result;
}
