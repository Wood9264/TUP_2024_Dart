#include "screen_task.h"
#include "screen_task_enum.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "tjc_usart_hmi.h"
#include "revolver_task.h"
#include "usart.h"

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
