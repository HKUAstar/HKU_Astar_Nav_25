#include "Strategy_Task.h"
#include "Referee_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "USB_Task.h"
#include "struct_typedef.h"

// #define DEBUG_STRATEGY_TASK

bool_t game_in_progress(void);
bool_t game_not_start_end(void);
CentralOccupation get_central_occupation(void);

void update_damaged(void);
bool_t check_damaged(void);
bool_t check_central_occupiable(void);
void update_central_occupiable(void);
void update_score(void);
void change_action(Action next_action);

bool_t is_sentry_recovered(void);
bool_t is_sentry_healthy(void);
bool_t is_sentry_in_danger(void);
bool_t is_sentry_dead(void);

Action action = INIT;
Motion motion = STAY_IN_PLACE;
Vision vision = NO_TARGET;

gimbal_operator_t gimbal_operator;
extern referee_info_t referee_info;
extern vision_rx_t vision_rx;
extern navigation_rx_t navigation_rx;

uint16_t central_occupiable_cnt = 0;
int16_t respawn_waiting_cnt = 0;
uint16_t check_occupation_cnt = 0;
uint16_t check_arrive_cnt = 0;
uint8_t check_arrive;
int16_t current_hp, previous_hp;
int16_t cumulative_damage = 0;
uint8_t current_red_dead = 0, previous_red_dead = 0;
uint8_t current_blue_dead = 0, previous_blue_dead = 0;
int16_t friendly_score = 0;
int16_t enemy_score = 0;

bool_t is_aggressive=0;

#ifdef DEBUG_STRATEGY_TASK
    referee_info_t referee_info_debug;
    CentralOccupation central_occupation_debug;
#endif

void Strategy_Task(void const *argument)
{   
    vTaskDelay(400);
    #ifdef DEBUG_STRATEGY_TASK
        referee_info_debug.remain_hp = 400;
        referee_info_debug.game_progress = 0;
        CentralOccupation central_occupation_debug = UNOCCUPIED;
    #endif

    while (1)
    {   
        switch (action)
        {
        case INIT:
            // 比赛开始占领中央
            if (game_in_progress())
            {
                change_action(PUSH);
            }
            break;

        case SIDEATTACK:

            // 比赛结束回到初始状态
            if (game_not_start_end())
            {
                change_action(INIT);
            }

            // 血量为0 进入死亡复活流程
            else if (is_sentry_dead())
            {
                change_action(RESPAWN);
            }

            // 血量严重不足撤退 回家回血
            else if (is_sentry_in_danger())
            {
                change_action(SUPPLY);
            }

            else
            {
                // 更新并检测中央是否可占领（连续多秒未受到攻击且中央未被占领）
                update_central_occupiable();
                if (check_central_occupiable())
                {
                    change_action(OCCUPY);
                }
            }

            break;

        case PUSH:
            // 血量不足撤退 侧面攻击
            // if (!is_sentry_healthy())
            // {
            //     change_action(SIDEATTACK);
            // }

            // 比赛结束回到初始状态
            if (game_not_start_end())
            {
                change_action(INIT);
            }

            // 血量为0 进入死亡复活流程
            else if (is_sentry_dead())
            {
                change_action(RESPAWN);
            }

            // 血量严重不足撤退 回家回血
            else if (is_sentry_in_danger())
            {
                change_action(SUPPLY);
            }

            // 弹丸不足但血量健康 去中央
            else if (!is_sentry_bullet_sufficient())
            {
                change_action(OCCUPY);
            }

            // 早期获得优势，占领中央
            else if (is_aggressive && friendly_score - enemy_score >= 50)
            {
                change_action(OCCUPY);
            }

            break;

        case OCCUPY:

            // 比赛结束回到初始状态
            if (game_not_start_end())
            {
                change_action(INIT);
            }

            // 血量为0 进入死亡复活流程
            else if (is_sentry_dead())
            {
                change_action(RESPAWN);
            }

            // 血量严重不足撤退 回家回血
            else if (is_sentry_in_danger())
            {
                change_action(SUPPLY);
            }

            // 比较劣势激进且弹丸充足
            else if (is_aggressive && friendly_score < enemy_score && is_sentry_bullet_sufficient())
            {
                change_action(PUSH);
            }

            // 更新并检测是否受到攻击
            // 受到攻击撤退 侧面攻击
            // update_damaged();
            // if (check_damaged())
            // {
            //     change_action(SIDEATTACK);
            // }

            break;

        case SUPPLY:

            // 比赛结束 回到初始状态
            if (game_not_start_end())
            {
                change_action(INIT);
            }

            // 血量为0 进入死亡复活流程
            else if (is_sentry_dead())
            {
                change_action(RESPAWN);
            }

            // 血量恢复 绕后攻击或占领中点
            else if (is_sentry_recovered())
            {
                // 弹丸充足且中点被占 绕后攻击
                if (get_central_occupation() == ENEMY_OCCUPIED && is_sentry_bullet_sufficient())
                {
                    change_action(PUSH);
                }
                // 弹丸不足或中点在手 前往中点
                else
                {
                    change_action(OCCUPY);
                }
            }

            break;
        case RESPAWN:
            // 比赛结束 回到初始状态
            if (game_not_start_end())
            {
                change_action(INIT);
            }

            // 等待结束 进入补给流程
            else if (respawn_waiting_cnt <= 0)
            {
                change_action(SUPPLY);
            }

            // 到达补给区 开始等待
            else if (check_arrive)
            {
                respawn_waiting_cnt--;
            }

            break;

        default:
            // 不应该到达这里 以防万一 恢复初始状态
            change_action(INIT);
            break;
        }
        previous_hp = current_hp;
        current_hp = referee_info.remain_hp;
        check_arrive_cnt++;
        check_arrive = (check_arrive_cnt % CHECK_ARRIVE_SPAN == 0) ? navigation_rx.arrived : check_arrive; // 每过10cycle检测一次是否到达目标点
        update_score();
        // decide low health enemies and prioritize them

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bool_t game_in_progress(void)
{
/* 0 未开始；1 准备阶段；2 15s裁判系统自检；3 5s倒计时；4 比赛中；5 比赛结算*/
#ifdef DEBUG_STRATEGY_TASK
    return referee_info_debug.game_progress == 4;
#else
    return referee_info.game_progress == 4;
#endif
}

bool_t game_rcheck_countdown_progress()
{
    /* 0 未开始；1 准备阶段；2 15s裁判系统自检；3 5s倒计时；4 比赛中；5 比赛结算*/
#ifdef DEBUG_STRATEGY_TASK
    if (referee_info_debug.game_progress == 2 || referee_info_debug.game_progress == 3 || referee_info_debug.game_progress == 4)
    {
        return 1;
    }
    else
    {
        return 0;
    }
#else
    if (referee_info.game_progress == 2 || referee_info.game_progress == 3 || referee_info.game_progress == 4)
    {
        return 1;
    }
    else
    {
        return 0;
    }
#endif
}

bool_t game_not_start_end()
{
    /*
    0 未开始；
    1 准备阶段；
    2 15s裁判系统自检；
    3 5s倒计时；
    4 比赛中；
    5 比赛结算*/
    #ifdef DEBUG_STRATEGY_TASK
        return referee_info_debug.game_progress == 0 || referee_info_debug.game_progress == 5;
    #else
        return referee_info.game_progress == 0 || referee_info.game_progress == 5;
    #endif
}

CentralOccupation get_central_occupation(void)
{
    #ifdef DEBUG_STRATEGY_TASK
        return (CentralOccupation) central_occupation_debug;
    #else
        return (CentralOccupation) referee_info.occupy_status;
    #endif
}

void update_damaged(void)
{
    int16_t decreased_hp;
    decreased_hp = previous_hp - current_hp;
    cumulative_damage += decreased_hp;
}
bool_t check_damaged(void)
{
    return cumulative_damage >= DAMAGE_LIMIT;
}
bool_t check_central_occupiable(void)
{
    return central_occupiable_cnt >= CENTRAL_OCCUPIABLE_SPAN;
}
void update_central_occupiable(void)
{
    if (get_central_occupation() != ENEMY_OCCUPIED && !check_damaged())
    {
        central_occupiable_cnt++;
    }
}
void update_score(void)
{
    if (referee_info.game_progress != 4) {
        previous_blue_dead = current_blue_dead;
        previous_red_dead = current_red_dead;
        current_blue_dead = referee_info.blue_dead;
        current_red_dead = referee_info.red_dead;
        return;      
    }
    // 占领中央增益点所得分数
    if (check_occupation_cnt % CHECK_OCCUPATION_SPAN == 0)
    {
        if (get_central_occupation() == ENEMY_OCCUPIED )
        {
            enemy_score++;
        }
        if (get_central_occupation() == FRIENDLY_OCCUPIED)
        {
            friendly_score++;
        }
    }

    // 击杀所得分数
    for (int index = 0; index < 5; index++) // 目前机器人数是5
    {
        bool_t previous_blue_robot_dead = previous_blue_dead >> index & 0x01;
        bool_t current_blue_robot_dead = current_blue_dead >> index & 0x01;
        if (previous_blue_robot_dead == 0 && current_blue_robot_dead == 1)
        {
            // 己方是蓝色
            if (referee_info.robot_color == 2)
            {
                enemy_score += 20;
            }
            // 己方是红色
            else
            {
                friendly_score += 20;
            }
        }

        bool_t previous_red_robot_dead = previous_red_dead >> index & 0x01;
        bool_t current_red_robot_dead = current_red_dead >> index & 0x01;
        if (previous_red_robot_dead == 0 && current_red_robot_dead == 1)
        {
            // 己方是蓝色
            if (referee_info.robot_color == 2)
            {
                friendly_score += 20;
            }
            // 己方是红色
            else
            {
                enemy_score += 20;
            }
        }


    }
    previous_blue_dead = current_blue_dead;
    previous_red_dead = current_red_dead;

#ifdef DEBUG_STRATEGY_TASK
    current_blue_dead = referee_info_debug.blue_dead;
    current_red_dead = referee_info_debug.red_dead;
#else
    current_blue_dead = referee_info.blue_dead;
    current_red_dead = referee_info.red_dead;
#endif
}

void change_action(Action next_action)
{

    action = next_action;
    cumulative_damage = 0;
    switch (action)
    {
    case INIT:
        check_arrive_cnt = 0;
        central_occupiable_cnt = 0;
        previous_hp = referee_info.remain_hp;
        current_hp = referee_info.remain_hp;
        motion = STAY_IN_PLACE;
        vision = NO_TARGET;
        break;

    case SIDEATTACK:
        motion = FRIENDLY_SIDEATTACK_LEFT;
        vision = ANY_TARGET;
        break;

    case PUSH:
        if(is_aggressive) {
            if(enemy_score - friendly_score >= 50) {
                motion = FRIENDLY_OBSTACLE_SPIN;
            }else {
                motion = ENEMY_OBSTACLE_SPIN;
            }
        } else {
            motion = CENTER_SPIN;
        }
        vision = ANY_TARGET;
        break;

    case OCCUPY:
        motion = CENTER_SPIN;
        vision = ANY_TARGET;
        central_occupiable_cnt = 0;
        break;

    case SUPPLY:
        motion = FRIENDLY_SUPPLY;
        vision = ANY_TARGET;
        break;

    case RESPAWN:
        motion = FRIENDLY_SUPPLY;
        vision = ANY_TARGET;
        respawn_waiting_cnt = RESPAWN_WAITING_SPAN;
        break;

    default:
        break;
    }
}

bool_t is_sentry_recovered(void)
{   
    #ifdef DEBUG_STRATEGY_TASK
        return referee_info_debug.remain_hp >= RECOVERED_HP;
    #else
        return (referee_info.remain_hp >= RECOVERED_HP);
    #endif
}

bool_t is_sentry_healthy(void)
{   
    #ifdef DEBUG_STRATEGY_TASK
        return referee_info_debug.remain_hp >= HEALTHY_HP;
    #else
        return (referee_info.remain_hp >= HEALTHY_HP);
    #endif
}

bool_t is_sentry_in_danger(void)
{   
    #ifdef DEBUG_STRATEGY_TASK
        return referee_info_debug.remain_hp < IN_DANGER_HP;
    #else
        return (referee_info.remain_hp < IN_DANGER_HP);
    #endif
}

bool_t is_sentry_dead(void)
{
#ifdef DEBUG_STRATEGY_TASK
    return referee_info_debug.remain_hp == 0;
#else
    return referee_info.remain_hp == 0;
#endif
}

bool_t is_sentry_bullet_sufficient(void)
{
#ifdef DEBUG_STRATEGY_TASK
    return referee_info_debug.bullet_remain >= BULLET_LIMIT;
#else
    return referee_info.bullet_remain >= BULLET_LIMIT;
#endif
}

uint8_t get_motion(void)
{
    return (uint8_t)motion;
}

uint8_t get_vision(void)
{
    return (uint8_t)vision;
}