#include "Referee_Task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "USB_Task.h"

referee_info_t referee_info;

// 云台上板，不负责处理裁判系统！由下板转发上来的
void Referee_Task(void const * argument)
{
    while(1)
    {
        // data_unpack();
        vTaskDelay(1);
    }
}

// void data_unpack(void)
// {
//     referee_info.robot_id = referee_info_buf[0];
//     referee_info.robot_color = get_robot_color(referee_info.robot_id);
// 	referee_info.game_progress = referee_info_buf[1];
// 	referee_info.stage_remain_time = referee_info_buf[2] << 8 | referee_info_buf[3];
// 	referee_info.remain_hp = referee_info_buf[4] << 8 | referee_info_buf[5];
// 	referee_info.max_hp = referee_info_buf[6] << 8 | referee_info_buf[7];

// 	referee_info.max_power = referee_info_buf[8] << 8 | referee_info_buf[9];
// 	referee_info.heat_limit = referee_info_buf[10] << 8 | referee_info_buf[11];
// 	referee_info.heat = referee_info_buf[12] << 8 | referee_info_buf[13];
// 	referee_info.bullet_speed = ((fp32) ((uint16_t)(referee_info_buf[14])));  
// 	referee_info.shooter_speed_limit = referee_info_buf[15];

// 	referee_info.red_1_hp = referee_info_buf[16] << 8 | referee_info_buf[17];
// 	referee_info.red_2_hp = referee_info_buf[18] << 8 | referee_info_buf[19];
// 	referee_info.red_3_hp = referee_info_buf[20] << 8 | referee_info_buf[21];
// 	referee_info.red_4_hp = referee_info_buf[22] << 8 | referee_info_buf[23];

// 	referee_info.blue_1_hp = referee_info_buf[24] << 8 | referee_info_buf[25];
// 	referee_info.blue_2_hp = referee_info_buf[26] << 8 | referee_info_buf[27];
// 	referee_info.blue_3_hp = referee_info_buf[28] << 8 | referee_info_buf[29];
// 	referee_info.blue_4_hp = referee_info_buf[30] << 8 | referee_info_buf[31];

// 	referee_info.red_5_hp = referee_info_buf[32] << 8 | referee_info_buf[33];
// 	referee_info.red_7_hp = referee_info_buf[34] << 8 | referee_info_buf[35];
// 	referee_info.blue_5_hp = referee_info_buf[36] << 8 | referee_info_buf[37];
// 	referee_info.blue_7_hp = referee_info_buf[38] << 8 | referee_info_buf[39];

// 	referee_info.red_base_hp = referee_info_buf[40] << 8 | referee_info_buf[41];
// 	referee_info.blue_base_hp = referee_info_buf[42] << 8 | referee_info_buf[43];
//     // referee_info.red_outpost_hp = referee_info_buf[44] << 8 | referee_info_buf[45];
//     // referee_info.blue_outpost_hp = referee_info_buf[46] << 8 | referee_info_buf[47];

// 	referee_info.event_data = (referee_info_buf[48] << 24 | referee_info_buf[49] << 16 | referee_info_buf[50] << 8 | referee_info_buf[51]);
//     // referee_info.bullet_remain = (referee_info_buf[52] << 8 | referee_info_buf[53]);
//     referee_info.gold_coin_remain = (referee_info_buf[54] << 8 | referee_info_buf[55]);

//     referee_info.radar_robot_id = referee_info_buf[56];
//     referee_info.radar_x = (float)(referee_info_buf[57] << 8 | referee_info_buf[58]) / 100;
//     referee_info.radar_y = (float)(referee_info_buf[59] << 8 | referee_info_buf[60]) / 100;
// 	// referee_info.rfid_patrol = referee_info_buf[61];
// }
