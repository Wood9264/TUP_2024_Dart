#include "screen_task.h"
#include "screen_task_enum.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "tjc_usart_hmi.h"
#include "revolver_task.h"
#include "usart.h"
#include "load_task.h"

screen_t screen;

screen_t *screen_point(void)
{
    return &screen;
}

void screen_task(void const *pvParameters)
{
    vTaskDelay(SCREEN_TASK_INIT_TIME);
    //串口屏显示数据初始化
    screen.screen_data_init();

    while (1)
    {
        //向串口屏发送数据
        screen.send_data();
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
 * @brief   串口屏显示数据初始化
 */
void screen_t::screen_data_init()
{
    //清空屏幕串口缓冲区里的无关数据，消除上电过程中引脚杂波的影响
    uint8_t init_data[] = {0x00, 0xff, 0xff, 0xff};
    HAL_UART_Transmit(&huart1, init_data, 4, 100);
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET)
        ;
    // //初始化转速补偿
    // TJCPrintf("t21.txt=\"%d\"", revolver_point()->fric_wheel_group.outpost_speed_offset[0]);
    // TJCPrintf("t22.txt=\"%d\"", revolver_point()->fric_wheel_group.outpost_speed_offset[1]);
    // TJCPrintf("t23.txt=\"%d\"", revolver_point()->fric_wheel_group.outpost_speed_offset[2]);
    // TJCPrintf("t24.txt=\"%d\"", revolver_point()->fric_wheel_group.outpost_speed_offset[3]);

    // // 初始化yaw轴补偿
    // TJCPrintf("t29.txt=\"%.2f\"", revolver_point()->yaw_motor.outpost_offset_num[0]);
    // TJCPrintf("t30.txt=\"%.2f\"", revolver_point()->yaw_motor.outpost_offset_num[1]);
    // TJCPrintf("t31.txt=\"%.2f\"", revolver_point()->yaw_motor.outpost_offset_num[2]);
    // TJCPrintf("t32.txt=\"%.2f\"", revolver_point()->yaw_motor.outpost_offset_num[3]);
}

/**
 * @brief   向串口屏发送数据
 */
void screen_t::send_data()
{
    // //发送电机转速
    // TJCPrintf("t5.txt=\"%d\"", revolver_point()->fric_wheel_group.fric_motor[0].motor_measure->speed_rpm);
    // TJCPrintf("t6.txt=\"%d\"", revolver_point()->fric_wheel_group.fric_motor[1].motor_measure->speed_rpm);
    // TJCPrintf("t7.txt=\"%d\"", revolver_point()->fric_wheel_group.fric_motor[2].motor_measure->speed_rpm);
    // TJCPrintf("t8.txt=\"%d\"", revolver_point()->fric_wheel_group.fric_motor[3].motor_measure->speed_rpm);

    // //发送电机温度
    // TJCPrintf("t9.txt=\"%d\"", revolver_point()->fric_wheel_group.fric_motor[0].motor_measure->temperate);
    // TJCPrintf("t10.txt=\"%d\"", revolver_point()->fric_wheel_group.fric_motor[1].motor_measure->temperate);
    // TJCPrintf("t11.txt=\"%d\"", revolver_point()->fric_wheel_group.fric_motor[2].motor_measure->temperate);
    // TJCPrintf("t12.txt=\"%d\"", revolver_point()->fric_wheel_group.fric_motor[3].motor_measure->temperate);

    // //发送转速设定值
    // TJCPrintf("t13.txt=\"%d\"", BASE_SPEED + revolver_point()->fric_wheel_group.outpost_speed_offset[0]);
    // TJCPrintf("t14.txt=\"%d\"", BASE_SPEED + revolver_point()->fric_wheel_group.outpost_speed_offset[1]);
    // TJCPrintf("t15.txt=\"%d\"", BASE_SPEED + revolver_point()->fric_wheel_group.outpost_speed_offset[2]);
    // TJCPrintf("t16.txt=\"%d\"", BASE_SPEED + revolver_point()->fric_wheel_group.outpost_speed_offset[3]);

    // //发送转速补偿
    // TJCPrintf("t17.txt=\"%d\"", revolver_point()->fric_wheel_group.outpost_speed_offset[0]);
    // TJCPrintf("t18.txt=\"%d\"", revolver_point()->fric_wheel_group.outpost_speed_offset[1]);
    // TJCPrintf("t19.txt=\"%d\"", revolver_point()->fric_wheel_group.outpost_speed_offset[2]);
    // TJCPrintf("t20.txt=\"%d\"", revolver_point()->fric_wheel_group.outpost_speed_offset[3]);

    // // 发送yaw轴补偿
    // TJCPrintf("t25.txt=\"%.2f\"", revolver_point()->yaw_motor.outpost_offset_num[0]);
    // TJCPrintf("t26.txt=\"%.2f\"", revolver_point()->yaw_motor.outpost_offset_num[1]);
    // TJCPrintf("t27.txt=\"%.2f\"", revolver_point()->yaw_motor.outpost_offset_num[2]);
    // TJCPrintf("t28.txt=\"%.2f\"", revolver_point()->yaw_motor.outpost_offset_num[3]);
}

/**
 * @brief   串口屏数据解析
 */
void screen_t::data_analysis()
{
    //帧内容
    uint8_t frame_content[MAX_FRAME_LENGTH] = {0};
    //单个字节数据
    uint8_t data;

    //判断缓冲区中是否有足够的数据
    if (getRingBuffLenght() >= MIN_FRAME_LENGTH)
    {
        //遍历缓冲区寻找帧头并删除帧头之前的错误数据
        for (uint8_t i = 0; i < RINGBUFF_LEN; i++)
        {
            data = read1BFromRingBuff(0);

            if (data == SCREEN_FRAME_HEADER)
            {
                //遍历后续内容寻找连续三个0xFF
                for (uint8_t j = 1; j < MAX_FRAME_LENGTH; j++)
                {
                    data = read1BFromRingBuff(j);

                    if (data == SCREEN_FRAME_TAIL)
                    {
                        if (read1BFromRingBuff(j + 1) == SCREEN_FRAME_TAIL && read1BFromRingBuff(j + 2) == SCREEN_FRAME_TAIL)
                        {
                            //解析帧内容
                            frame_content_analysis(frame_content);
                            //删除已解析的帧
                            deleteRingBuff(j + 3);
                            //递归调用，继续解析下一帧
                            data_analysis();
                            return;
                        }
                        else
                        {
                            frame_content[j - 1] = data;
                        }
                    }
                    //如果读取到下一个帧头，说明本帧的帧尾丢失，删除本帧
                    else if (data == SCREEN_FRAME_HEADER)
                    {
                        deleteRingBuff(j);
                        return;
                    }
                    else
                    {
                        frame_content[j - 1] = data;
                    }
                }
                //如果遍历完最大长度的帧内容后没有找到帧尾，说明帧内容错误，递归调用继续解析下一帧
                data_analysis();
                return;
            }
            else
            {
                deleteRingBuff(1);
            }
        }
    }
}

/**
 * @brief   帧内容解析
 * @param   frame_content 帧内容
 */
void screen_t::frame_content_analysis(uint8_t *frame_content)
{
    uint8_t cmd_ID = 0;

    //获取命令ID
    cmd_ID = frame_content[0];

    switch (cmd_ID)
    {
    case ID_outpost_speed_1:
        outpost_speed[0] = ascii_to_int16_t(frame_content + CMD_ID_LENTH);
        break;
    case ID_outpost_speed_2:
        outpost_speed[1] = ascii_to_int16_t(frame_content + CMD_ID_LENTH);
        break;
    case ID_outpost_speed_3:
        outpost_speed[2] = ascii_to_int16_t(frame_content + CMD_ID_LENTH);
        break;
    case ID_outpost_speed_4:
        outpost_speed[3] = ascii_to_int16_t(frame_content + CMD_ID_LENTH);
        break;
    case ID_base_speed_1:
        base_speed[0] = ascii_to_int16_t(frame_content + CMD_ID_LENTH);
        break;
    case ID_base_speed_2:
        base_speed[1] = ascii_to_int16_t(frame_content + CMD_ID_LENTH);
        break;
    case ID_base_speed_3:
        base_speed[2] = ascii_to_int16_t(frame_content + CMD_ID_LENTH);
        break;
    case ID_base_speed_4:
        base_speed[3] = ascii_to_int16_t(frame_content + CMD_ID_LENTH);
        break;

    case ID_outpost_yaw_offset_num_1:
        outpost_yaw_offset_num[0] = ascii_to_fp32(frame_content + CMD_ID_LENTH);
        break;
    case ID_outpost_yaw_offset_num_2:
        outpost_yaw_offset_num[1] = ascii_to_fp32(frame_content + CMD_ID_LENTH);
        break;
    case ID_outpost_yaw_offset_num_3:
        outpost_yaw_offset_num[2] = ascii_to_fp32(frame_content + CMD_ID_LENTH);
        break;
    case ID_outpost_yaw_offset_num_4:
        outpost_yaw_offset_num[3] = ascii_to_fp32(frame_content + CMD_ID_LENTH);
        break;
    case ID_base_yaw_offset_num_1:
        base_yaw_offset_num[0] = ascii_to_fp32(frame_content + CMD_ID_LENTH);
        break;
    case ID_base_yaw_offset_num_2:
        base_yaw_offset_num[1] = ascii_to_fp32(frame_content + CMD_ID_LENTH);
        break;
    case ID_base_yaw_offset_num_3:
        base_yaw_offset_num[2] = ascii_to_fp32(frame_content + CMD_ID_LENTH);
        break;
    case ID_base_yaw_offset_num_4:
        base_yaw_offset_num[3] = ascii_to_fp32(frame_content + CMD_ID_LENTH);
        break;

    case ID_refresh:
        refresh_data();
        break;
    case ID_fric_monitor:
        fric_monitor(frame_content + CMD_ID_LENTH);
        break;
    case ID_other_monitor:
        other_monitor(frame_content + CMD_ID_LENTH);
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
