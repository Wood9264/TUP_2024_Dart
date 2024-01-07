#include "screen_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "tjc_usart_hmi.h"
#include "revolver_task.h"

screen_t screen;

void screen_task(void const *pvParameters)
{
    vTaskDelay(2000);

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
 * @brief 串口屏构造函数
 */
screen_t::screen_t()
{
    initRingBuff();
}

/**
 * @brief 向串口屏发送数据
 */
void screen_t::send_data()
{
    //发送电机转速
    TJCPrintf("t5.txt=\"%d\"", revolver_point()->fric_motor[0].motor_measure->speed_rpm);
    TJCPrintf("t6.txt=\"%d\"", revolver_point()->fric_motor[1].motor_measure->speed_rpm);
    TJCPrintf("t7.txt=\"%d\"", revolver_point()->fric_motor[2].motor_measure->speed_rpm);
    TJCPrintf("t8.txt=\"%d\"", revolver_point()->fric_motor[3].motor_measure->speed_rpm);

    //发送电机温度
    TJCPrintf("t9.txt=\"%d\"", revolver_point()->fric_motor[0].motor_measure->temperate);
    TJCPrintf("t10.txt=\"%d\"", revolver_point()->fric_motor[1].motor_measure->temperate);
    TJCPrintf("t11.txt=\"%d\"", revolver_point()->fric_motor[2].motor_measure->temperate);
    TJCPrintf("t12.txt=\"%d\"", revolver_point()->fric_motor[3].motor_measure->temperate);

    //发送转速设定值
    TJCPrintf("t13.txt=\"%d\"", BASE_SPEED + revolver_point()->outpost_speed_offset[0]);
    TJCPrintf("t14.txt=\"%d\"", BASE_SPEED + revolver_point()->outpost_speed_offset[1]);
    TJCPrintf("t15.txt=\"%d\"", BASE_SPEED + revolver_point()->outpost_speed_offset[2]);
    TJCPrintf("t16.txt=\"%d\"", BASE_SPEED + revolver_point()->outpost_speed_offset[3]);

    //发送转速补偿
    TJCPrintf("t17.txt=\"%d\"", revolver_point()->outpost_speed_offset[0]);
    TJCPrintf("t18.txt=\"%d\"", revolver_point()->outpost_speed_offset[1]);
    TJCPrintf("t19.txt=\"%d\"", revolver_point()->outpost_speed_offset[2]);
    TJCPrintf("t20.txt=\"%d\"", revolver_point()->outpost_speed_offset[3]);

    // 发送yaw轴补偿
    TJCPrintf("t25.txt=\"%.2f\"", revolver_point()->yaw_motor.offset_num[0]);
    TJCPrintf("t26.txt=\"%.2f\"", revolver_point()->yaw_motor.offset_num[1]);
    TJCPrintf("t27.txt=\"%.2f\"", revolver_point()->yaw_motor.offset_num[2]);
    TJCPrintf("t28.txt=\"%.2f\"", revolver_point()->yaw_motor.offset_num[3]);
}

uint8_t CONTENT[HMI_USART_RX_BUF_LENGHT];

/**
 * @brief 串口屏数据解析
 */
void screen_t::data_analysis()
{
    //帧内容
    uint8_t frame_content[HMI_USART_RX_BUF_LENGHT];
    uint8_t data;

    //判断缓冲区中是否有数据
    if (getRingBuffLenght() > 0)
    {

        if (read1BFromRingBuff(0) == SCREEN_FRAME_HEADER)
        {
            deleteRingBuff(1);

            //逐个读取数据并存入帧内容数组，直到读取到连续三个0xFF
            for (uint8_t i = 0; i < HMI_USART_RX_BUF_LENGHT; i++)
            {
                data = read1BFromRingBuff(0);

                if (data == SCREEN_FRAME_TAIL)
                {
                    if (read1BFromRingBuff(1) == SCREEN_FRAME_TAIL && read1BFromRingBuff(2) == SCREEN_FRAME_TAIL)
                    {
                        deleteRingBuff(3);
                        //解析帧内容
                        //...
                        
                        break;
                    }
                    else
                    {
                        frame_content[i] = data;
                        CONTENT[i] = data;
                        deleteRingBuff(1);
                    }
                }
                else
                {
                    frame_content[i] = data;
                    CONTENT[i] = data;
                    deleteRingBuff(1);
                }
            }
        }
        else
        {
            deleteRingBuff(1);
        }
    }
}
