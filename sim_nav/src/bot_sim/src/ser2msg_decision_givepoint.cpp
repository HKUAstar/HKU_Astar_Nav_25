#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <vector>
//新增头文件
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

const double PI = 3.14159265358979323846;

serial::Serial ser;

// 裁判系统通讯常量（根据rm通讯协议）
const uint8_t REFEREE_SOF = 0xA5;
const uint16_t REFEREE_MAX_PACKET_LENGTH = 256;
const int REFEREE_BAUDRATE = 115200;

//裁判系统串口和解析器 
serial::Serial referee_ser;           // 裁判系统串口
bool referee_connected = false;       // 裁判系统连接状态
RefereeParser* referee_parser = nullptr; // 裁判系统解析器指针

const int write_length = 15;
const int read_length = 19;

geometry_msgs::TransformStamped transformRotbaseToVirtual;
geometry_msgs::TransformStamped transformGimbalToRotbase;
geometry_msgs::TransformStamped transformMapToGimbal;
geometry_msgs::TransformStamped loc;
bool status = 0;

union FloatToByte{
    float f;
    uint8_t bytes[sizeof(float)];
};

union ByteToByte{
    uint8_t f;
    uint8_t bytes[sizeof(float)];
};

//裁判系统数据结构
#pragma pack(push, 1)

// 帧头结构
struct FrameHeader {
    uint8_t sof;            // 固定 0xA5
    uint16_t data_length;   // data段长度
    uint8_t seq;            // 包序号
    uint8_t crc8;           // 帧头 CRC8
};

// 比赛状态数据
struct GameStatusData {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
};

// 比赛结果数据
struct GameResultData {
    uint8_t winner;  // 0:平局, 1:红胜, 2:蓝胜
};

// 机器人血量数据
struct GameRobotHPData {
    uint16_t ally_1_robot_HP;   // 英雄
    uint16_t ally_2_robot_HP;   // 工程
    uint16_t ally_3_robot_HP;   // 步兵3
    uint16_t ally_4_robot_HP;   // 步兵4
    uint16_t reserved;          // 保留
    uint16_t ally_7_robot_HP;   // 哨兵
    uint16_t ally_output_HP;    // 前哨站
    uint16_t ally_base_HP;      // 基地
};

// 场地事件数句
struct EventData {
    uint32_t event_data;  
};

// 机器人性能体系数据
struct RobotStatusData {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
    uint8_t reserved : 5;  
};

// 机器人位置数据
struct RobotPosData {
    float x;       
    float y;       
    float angle;   
};

// 允许发弹量数据
struct ProjectileAllowanceData {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
};

#pragma pack(pop)
// 裁判系统数据解析
class RefereeParser {
private:
    // ROS发布给决策
    ros::Publisher game_progress_pub_;
    ros::Publisher stage_remain_time_pub_;
    ros::Publisher robot_hp_pub_;           
    ros::Publisher robot_max_hp_pub_;
    ros::Publisher bullet_remain_pub_;      
    ros::Publisher friendly_score_pub_;     
    ros::Publisher enemy_score_pub_;       
    ros::Publisher bullet_42mm_pub_;
    ros::Publisher robot_pos_pub_;
    ros::Publisher robot_angle_pub_;
    ros::Publisher robot_id_pub_;
    ros::Publisher robot_team_pub_;        // 0=未知, 1=红方, 2=蓝方
    ros::Publisher central_occupation_pub_; // 中央占领状态
    

    uint8_t robot_id_;            // 机器人ID
    uint8_t robot_color_;         // 机器人颜色: 0=未知, 1=红方, 2=蓝方
    uint8_t game_progress_;       // 0=未开始, 1=准备阶段, 2=自检, 3=倒计时, 4=比赛中, 5=结算
    uint16_t stage_remain_time_;
    uint16_t robot_hp_;
    uint16_t max_hp_;
    uint16_t bullet_remain_;      // 17mm弹量
    uint16_t bullet_42mm_;        // 42mm弹量
    float robot_x_;
    float robot_y_;
    float robot_angle_;
    
    // 场地状态
    uint32_t event_data_;         
    
    // 中央占领状态
    uint8_t occupy_status_;       // 0=未被占领, 1=红方占领, 2=蓝方占领
    
    // 双方机器人死亡状态（从血量数据推断）
    uint8_t red_dead_;            // 红方死亡状态（每位表示一个机器人）
    uint8_t blue_dead_;           // 蓝方死亡状态（每位表示一个机器人）
    uint8_t previous_red_dead_;
    uint8_t previous_blue_dead_;
    
    // 得分计算相关
    int friendly_score_;          // 己方得分（哨兵视角）
    int enemy_score_;             // 敌方得分（哨兵视角）
    uint16_t check_occupation_cnt_; // 占领检查计数器
    

    // 双方机器人血量（用于判断死亡）
    uint16_t red_1_hp_, red_2_hp_, red_3_hp_, red_4_hp_, red_5_hp_, red_7_hp_;
    uint16_t blue_1_hp_, blue_2_hp_, blue_3_hp_, blue_4_hp_, blue_5_hp_, blue_7_hp_;
    uint16_t red_base_hp_, blue_base_hp_;

    // 缓冲区
    std::vector<uint8_t> buffer_;
    
    // 常量定义
    static const int CHECK_OCCUPATION_SPAN = 200; // 200*50ms = 10秒
    static const uint8_t REFEREE_SOF = 0xA5;      // 协议定义的帧起始字节
    
    
    // 辅助函数
    uint8_t get_robot_color(uint8_t robot_id) {
        if (robot_id >= 1 && robot_id <= 11) return 1;   // 红方
        if (robot_id >= 101 && robot_id <= 111) return 2; // 蓝方
        return 0; // 未知
    }
    
    // 从event_data解析中央占领状态
    uint8_t parseCentralOccupation(uint32_t event_data) {
        // 根据协议表1-8: bit7-8为中央高地占领状态
        // 1:被己方占领, 2:被对方占领
        uint8_t occupation = (event_data >> 7) & 0x03;
        
        // 转换为通用格式: 0=未被占领, 1=红方占领, 2=蓝方占领
        // 注意: 这里的"己方"需要结合哨兵队伍判断
        if (occupation == 1) {
            return (robot_color_ == 1) ? 1 : 2; // 己方是红方->红方占领
        } else if (occupation == 2) {
            return (robot_color_ == 1) ? 2 : 1; // 对方是红方->蓝方占领
        }
        return 0; // 未被占领
    }
    
    // 更新机器人死亡状态（从血量判断）
    void updateRobotDeathStatus() {
        red_dead_ = 0;
        blue_dead_ = 0;
        
        // 红方机器人死亡判断
        if (red_1_hp_ == 0) red_dead_ |= (1 << 0); // 英雄
        if (red_2_hp_ == 0) red_dead_ |= (1 << 1); // 工程
        if (red_3_hp_ == 0) red_dead_ |= (1 << 2); // 3号步兵
        if (red_4_hp_ == 0) red_dead_ |= (1 << 3); // 4号步兵
        if (red_7_hp_ == 0) red_dead_ |= (1 << 4); // 哨兵
        
        // 蓝方机器人死亡判断
        if (blue_1_hp_ == 0) blue_dead_ |= (1 << 0); // 英雄
        if (blue_2_hp_ == 0) blue_dead_ |= (1 << 1); // 工程
        if (blue_3_hp_ == 0) blue_dead_ |= (1 << 2); // 3号步兵
        if (blue_4_hp_ == 0) blue_dead_ |= (1 << 3); // 4号步兵
        if (blue_7_hp_ == 0) blue_dead_ |= (1 << 4); // 哨兵
    }
    
    // 更新得分（通过裁判系统计算）（！！这个地方有可能需要根据新规则有改动）
    void updateScore() {
        // 只有比赛进行中才更新得分
        if (game_progress_ != 4) {
            previous_blue_dead_ = blue_dead_;
            previous_red_dead_ = red_dead_;
            return;
        }
        
        // 1. 占领中央增益点所得分数（每10秒加1分）
        if (check_occupation_cnt_ % CHECK_OCCUPATION_SPAN == 0) {
            if (occupy_status_ == 2 && robot_color_ == 1) { // 蓝方占领，己方是红方
                enemy_score_++;
            } else if (occupy_status_ == 2 && robot_color_ == 2) { // 蓝方占领，己方是蓝方
                friendly_score_++;
            } else if (occupy_status_ == 1 && robot_color_ == 1) { // 红方占领，己方是红方
                friendly_score_++;
            } else if (occupy_status_ == 1 && robot_color_ == 2) { // 红方占领，己方是蓝方
                enemy_score_++;
            }
            check_occupation_cnt_ = 0;
        }
        check_occupation_cnt_++;
        
        // 2. 击杀所得分数（击杀一个机器人得20分）
        for (int index = 0; index < 5; index++) { // 目前考虑5个机器人
            bool previous_blue_robot_dead = (previous_blue_dead_ >> index) & 0x01;
            bool current_blue_robot_dead = (blue_dead_ >> index) & 0x01;
            
            if (previous_blue_robot_dead == 0 && current_blue_robot_dead == 1) {
                // 己方是蓝方
                if (robot_color_ == 2) {
                    enemy_score_ += 20; // 敌方（红方）击杀了我方（蓝方）机器人
                }
                // 己方是红方
                else {
                    friendly_score_ += 20; // 敌方（蓝方）击杀了敌方（蓝方）机器人
                }
            }
            
            bool previous_red_robot_dead = (previous_red_dead_ >> index) & 0x01;
            bool current_red_robot_dead = (red_dead_ >> index) & 0x01;
            
            if (previous_red_robot_dead == 0 && current_red_robot_dead == 1) {
                // 己方是蓝方
                if (robot_color_ == 2) {
                    friendly_score_ += 20; // 敌方（红方）击杀了敌方（红方）机器人
                }
                // 己方是红方
                else {
                    enemy_score_ += 20; // 敌方（蓝方）击杀了我方（红方）机器人
                }
            }
        }
        
        previous_blue_dead_ = blue_dead_;
        previous_red_dead_ = red_dead_;
        
        // 发布更新后的得分
        publishScores();
    }
    
    // 发布得分
    void publishScores() {
        std_msgs::Int32 msg;
        msg.data = friendly_score_;
        friendly_score_pub_.publish(msg);
        
        msg.data = enemy_score_;
        enemy_score_pub_.publish(msg);
    }
    
    // 发布所有决策需要的信息
    void publishAllDecisionInfo() {
        std_msgs::Int32 int_msg;
        std_msgs::UInt8 byte_msg;
        std_msgs::Float32 float_msg;
        
        // 1. 比赛进度（决策模块需要）
        int_msg.data = game_progress_;
        game_progress_pub_.publish(int_msg);
        
        // 2. 阶段剩余时间
        int_msg.data = stage_remain_time_;
        stage_remain_time_pub_.publish(int_msg);
        
        // 3. 机器人血量（决策模块需要）
        int_msg.data = robot_hp_;
        robot_hp_pub_.publish(int_msg);
        
        int_msg.data = max_hp_;
        robot_max_hp_pub_.publish(int_msg);
        
        // 4. 弹量（决策模块需要）
        int_msg.data = bullet_remain_;
        bullet_remain_pub_.publish(int_msg);
        
        int_msg.data = bullet_42mm_;
        bullet_42mm_pub_.publish(int_msg);
        
        // 5. 机器人位置和角度
        float_msg.data = robot_x_;
        robot_pos_pub_.publish(float_msg);
        
        float_msg.data = robot_angle_;
        robot_angle_pub_.publish(float_msg);
        
        // 6. 机器人ID和队伍
        byte_msg.data = robot_id_;
        robot_id_pub_.publish(byte_msg);
        
        byte_msg.data = robot_color_; // 1=红方, 2=蓝方
        robot_team_pub_.publish(byte_msg);
        
        // 7. 中央占领状态
        byte_msg.data = occupy_status_;
        central_occupation_pub_.publish(byte_msg);
    }
    
    // 解析裁判系统数据包
    bool parseRefereePacket(const uint8_t* data, size_t length) {
        if (length < sizeof(FrameHeader)) {
            ROS_WARN("Referee packet too short: %zu bytes", length);
            return false;
        }
        
        FrameHeader* header = (FrameHeader*)data;
        
        // 验证SOF
        if (header->sof != REFEREE_SOF) {
            ROS_WARN("Invalid SOF in referee packet: 0x%02X", header->sof);
            return false;
        }
        
        // 检查数据长度
        size_t expected_length = sizeof(FrameHeader) + header->data_length + 2; // +2 for CRC16
        if (length < expected_length) {
            ROS_WARN("Incomplete referee packet: expected %zu, got %zu", expected_length, length);
            return false;
        }
        
        // 验证CRC8（帧头校验）
        uint8_t crc8 = Get_CRC8_Check_Sum(data, sizeof(FrameHeader) - 1, 0xFF);
        if (crc8 != header->crc8) {
            ROS_WARN("CRC8 mismatch in referee packet");
            return false;
        }
        
        // 获取命令ID和数据部分
        const uint8_t* cmd_data = data + sizeof(FrameHeader);
        uint16_t cmd_id = *((uint16_t*)cmd_data);
        const uint8_t* packet_data = cmd_data + 2; // 跳过cmd_id
        
        // 验证CRC16（整个数据包校验）
        uint16_t crc16 = Get_CRC16_Check_Sum(data, expected_length - 2, 0xFFFF);
        uint16_t received_crc16 = *((uint16_t*)(data + expected_length - 2));
        if (crc16 != received_crc16) {
            ROS_WARN("CRC16 mismatch in referee packet");
            return false;
        }
        
        // 根据命令ID解析数据
        parseRefereeData(cmd_id, packet_data, header->data_length - 2); // -2 for cmd_id
        
        return true;
    }
    
    // 解析具体的裁判数据（根据协议文档）
    void parseRefereeData(uint16_t cmd_id, const uint8_t* data, size_t length) {
        switch (cmd_id) {
            case 0x0001: // 比赛状态数据
                parseGameStatus(data, length);
                break;
            case 0x0002: // 比赛结果数据
                parseGameResult(data, length);
                break;
            case 0x0003: // 全体机器人血量数据（己方视角）
                parseAllRobotHP(data, length);
                break;
            case 0x0101: // 场地事件数据
                parseEventData(data, length);
                break;
            case 0x0201: // 机器人状态数据（自身）
                parseRobotStatus(data, length);
                break;
            case 0x0203: // 机器人位置数据
                parseRobotPosition(data, length);
                break;
            case 0x0208: // 弹丸剩余发射数
                parseProjectileAllowance(data, length);
                break;
            default:
                ROS_DEBUG("Unhandled referee command: 0x%04X", cmd_id);
                break;
        }
        
        // 每次解析完数据都更新得分并发布
        updateScore();
        publishAllDecisionInfo();
    }
    
    // 解析比赛状态数据（cmd_id: 0x0001）
    void parseGameStatus(const uint8_t* data, size_t length) {
        if (length < sizeof(GameStatusData)) {
            ROS_WARN("GameStatus data too short");
            return;
        }
        
        GameStatusData status_data;
        memcpy(&status_data, data, sizeof(GameStatusData));
        
        game_progress_ = status_data.game_progress;
        stage_remain_time_ = status_data.stage_remain_time;
        
        ROS_DEBUG_THROTTLE(1.0, "Game Progress: %d, Remaining Time: %d", 
                          game_progress_, stage_remain_time_);
    }
    
    // 解析比赛结果数据（cmd_id: 0x0002）
    void parseGameResult(const uint8_t* data, size_t length) {
        if (length < sizeof(GameResultData)) {
            ROS_WARN("GameResult data too short");
            return;
        }
        
        GameResultData result_data;
        memcpy(&result_data, data, sizeof(GameResultData));
        
        ROS_INFO("Game result: %s", 
                result_data.winner == 0 ? "Draw" : 
                result_data.winner == 1 ? "Red wins" : "Blue wins");
    }
    
    // 解析全体机器人血量数据（cmd_id: 0x0003）- 己方视角
    void parseAllRobotHP(const uint8_t* data, size_t length) {
        if (length < sizeof(GameRobotHPData)) {
            ROS_WARN("AllRobotHP data too short");
            return;
        }
        
        GameRobotHPData hp_data;
        memcpy(&hp_data, data, sizeof(GameRobotHPData));
        
        // 根据哨兵队伍更新血量
        if (robot_color_ == 1) { // 红方哨兵
            red_1_hp_ = hp_data.ally_1_robot_HP;
            red_2_hp_ = hp_data.ally_2_robot_HP;
            red_3_hp_ = hp_data.ally_3_robot_HP;
            red_4_hp_ = hp_data.ally_4_robot_HP;
            red_7_hp_ = hp_data.ally_7_robot_HP; // 哨兵自身血量
            red_base_hp_ = hp_data.ally_base_HP;
        } else if (robot_color_ == 2) { // 蓝方哨兵
            blue_1_hp_ = hp_data.ally_1_robot_HP;
            blue_2_hp_ = hp_data.ally_2_robot_HP;
            blue_3_hp_ = hp_data.ally_3_robot_HP;
            blue_4_hp_ = hp_data.ally_4_robot_HP;
            blue_7_hp_ = hp_data.ally_7_robot_HP; // 哨兵自身血量
            blue_base_hp_ = hp_data.ally_base_HP;
        }
        
        // 更新死亡状态
        updateRobotDeathStatus();
        
        ROS_DEBUG_THROTTLE(2.0, "All robot HP updated");
    }
    
    // 解析场地事件数据（cmd_id: 0x0101）
    void parseEventData(const uint8_t* data, size_t length) {
        if (length < sizeof(EventData)) {
            ROS_WARN("EventData too short");
            return;
        }
        
        EventData event_data_struct;
        memcpy(&event_data_struct, data, sizeof(EventData));
        event_data_ = event_data_struct.event_data;
        
        // 解析中央占领状态
        occupy_status_ = parseCentralOccupation(event_data_);
        
        ROS_DEBUG_THROTTLE(2.0, "Event data: 0x%08X, Central occupation: %d", 
                          event_data_, occupy_status_);
    }
    
    // 解析机器人状态数据（cmd_id: 0x0201）
    void parseRobotStatus(const uint8_t* data, size_t length) {
        if (length < sizeof(RobotStatusData)) {
            ROS_WARN("RobotStatus data too short");
            return;
        }
        
        RobotStatusData robot_status;
        memcpy(&robot_status, data, sizeof(RobotStatusData));
        
        // 首次接收到机器人状态，确定身份
        if (robot_id_ == 0) {
            robot_id_ = robot_status.robot_id;
            robot_color_ = get_robot_color(robot_id_);
            ROS_INFO("Sentry robot identified: ID=%d, Team=%s", 
                    robot_id_, robot_color_ == 1 ? "RED" : "BLUE");
        }
        
        // 只处理哨兵自身的数据
        if (robot_status.robot_id == robot_id_) {
            robot_hp_ = robot_status.current_HP;
            max_hp_ = robot_status.maximum_HP;
            
            ROS_DEBUG_THROTTLE(1.0, "Robot HP: %d/%d", robot_hp_, max_hp_);
        }
    }
    
    // 解析机器人位置数据（cmd_id: 0x0203）
    void parseRobotPosition(const uint8_t* data, size_t length) {
        if (length < sizeof(RobotPosData)) {
            ROS_WARN("RobotPosition data too short");
            return;
        }
        
        RobotPosData pos_data;
        memcpy(&pos_data, data, sizeof(RobotPosData));
        
        robot_x_ = pos_data.x;
        robot_y_ = pos_data.y;
        robot_angle_ = pos_data.angle;
        
        ROS_DEBUG_THROTTLE(1.0, "Robot Position: (%.2f, %.2f), Angle: %.2f", 
                          robot_x_, robot_y_, robot_angle_);
    }
    
    // 解析弹丸剩余发射数（cmd_id: 0x0208）
    void parseProjectileAllowance(const uint8_t* data, size_t length) {
        if (length < sizeof(ProjectileAllowanceData)) {
            ROS_WARN("ProjectileAllowance data too short");
            return;
        }
        
        ProjectileAllowanceData allowance_data;
        memcpy(&allowance_data, data, sizeof(ProjectileAllowanceData));
        
        bullet_remain_ = allowance_data.projectile_allowance_17mm;
        bullet_42mm_ = allowance_data.projectile_allowance_42mm;
        gold_coin_remain_ = allowance_data.remaining_gold_coin;
        
        ROS_DEBUG_THROTTLE(1.0, "Ammo: 17mm=%d, 42mm=%d, Gold: %d", 
                          bullet_remain_, bullet_42mm_, gold_coin_remain_);
    }
    
    
    // CRC校验函数（根据附录一）
    uint8_t Get_CRC8_Check_Sum(const uint8_t* pchMessage, size_t dwLength, uint8_t ucCRC8) {
        static const uint8_t CRC8_TAB[256] = {
            0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
            0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        };
        
        while (dwLength--) {
            ucCRC8 = CRC8_TAB[ucCRC8 ^ (*pchMessage++)];
        }
        return ucCRC8;
    }
    
    uint16_t Get_CRC16_Check_Sum(const uint8_t* pchMessage, size_t dwLength, uint16_t wCRC) {
        static const uint16_t wCRC_Table[256] = {
            0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
            0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        };
        
        while (dwLength--) {
            wCRC = (wCRC >> 8) ^ wCRC_Table[(wCRC ^ (*pchMessage++)) & 0xFF];
        }
        return wCRC;
    }
    
public:
    RefereeParser(ros::NodeHandle& nh, int default_sentry_id = 0, bool default_is_red = true) 
        : robot_id_(0),
          robot_color_(0),
          game_progress_(0),
          stage_remain_time_(0),
          robot_hp_(400),
          max_hp_(400),
          bullet_remain_(999),
          bullet_42mm_(0),
          robot_x_(0),
          robot_y_(0),
          robot_angle_(0),
          occupy_status_(0),
          red_dead_(0),
          blue_dead_(0),
          previous_red_dead_(0),
          previous_blue_dead_(0),
          friendly_score_(0),
          enemy_score_(0),
          check_occupation_cnt_(0)
    {
        // 初始化发布器
        game_progress_pub_ = nh.advertise<std_msgs::Int32>("/referee/game_progress", 10);
        stage_remain_time_pub_ = nh.advertise<std_msgs::Int32>("/referee/stage_remain_time", 10);
        robot_hp_pub_ = nh.advertise<std_msgs::Int32>("/referee/remain_hp", 10);
        robot_max_hp_pub_ = nh.advertise<std_msgs::Int32>("/referee/max_hp", 10);
        bullet_remain_pub_ = nh.advertise<std_msgs::Int32>("/referee/bullet_remain", 10);
        friendly_score_pub_ = nh.advertise<std_msgs::Int32>("/referee/friendly_score", 10);
        enemy_score_pub_ = nh.advertise<std_msgs::Int32>("/referee/enemy_score", 10);
        bullet_42mm_pub_ = nh.advertise<std_msgs::Int32>("/referee/bullet_42mm", 10);
        robot_pos_pub_ = nh.advertise<std_msgs::Float32>("/referee/robot_x", 10);
        robot_angle_pub_ = nh.advertise<std_msgs::Float32>("/referee/robot_angle", 10);
        robot_id_pub_ = nh.advertise<std_msgs::UInt8>("/referee/robot_id", 10);
        robot_team_pub_ = nh.advertise<std_msgs::UInt8>("/referee/robot_team", 10);
        central_occupation_pub_ = nh.advertise<std_msgs::UInt8>("/referee/central_occupation", 10);
        
        // 如果没有从裁判系统获取到ID，使用默认值
        if (default_sentry_id > 0) {
            robot_id_ = default_sentry_id;
            robot_color_ = default_is_red ? 1 : 2;
            ROS_INFO("RefereeParser initialized with default: ID=%d, Team=%s", 
                    robot_id_, robot_color_ == 1 ? "RED" : "BLUE");
        } else {
            ROS_INFO("RefereeParser initialized, waiting for referee data...");
        }
    }
    
    // 接收裁判系统数据
    void receiveData(const uint8_t* data, size_t length) {
        // 将数据添加到缓冲区
        buffer_.insert(buffer_.end(), data, data + length);
        
        // 尝试解析完整的数据包
        processBuffer();
    }
    
    // 处理缓冲区中的数据
    void processBuffer() {
        size_t offset = 0;
        
        while (offset + sizeof(FrameHeader) <= buffer_.size()) {
            FrameHeader* header = (FrameHeader*)&buffer_[offset];
            
            // 检查是否有完整的数据包
            size_t packet_size = sizeof(FrameHeader) + header->data_length + 2;
            if (offset + packet_size > buffer_.size()) {
                break; // 数据不完整，等待更多数据
            }
            
            // 解析数据包
            if (parseRefereePacket(&buffer_[offset], packet_size)) {
                // 成功解析，移动到下一个数据包
                offset += packet_size;
            } else {
                // 解析失败，跳过这个字节继续尝试
                offset += 1;
            }
        }
        
        // 删除已处理的数据
        if (offset > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + offset);
        }
    }
    
    // 获取哨兵是否是红方
    bool isRedTeam() const {
        return robot_color_ == 1;
    }
    
    // 获取哨兵是否是蓝方
    bool isBlueTeam() const {
        return robot_color_ == 2;
    }
    
};

class Message {
public:
    static const uint8_t SOF = 0x91;
    static const uint8_t eof = 0xFE;
    // float imu_change_threshold = 0.2;
    // float relative_change_threshold = 15;
    // float past_imu_angle;
    // float past_relative_angle;
    float imu_angle = 0;
    float relative_angle = 0; float goal_x = 0; float goal_y = 0;
    int goal_type = 13;
    bool receive_message = 0;
    std::vector<uint8_t> buffer;

    bool readFromBuffer() {
        if (this->buffer.size() < read_length) {
            // ROS_INFO("Not enough data available from the serial port, current buffer size: %d", buffer.size());
            return false; // Not enough data, 
        }
        else if (this->buffer[0] == SOF && this->buffer[read_length-1] == eof) {
            memcpy(&this->relative_angle, &this->buffer[1], sizeof(float));
            memcpy(&this->imu_angle, &this->buffer[5], sizeof(float));
            unsigned char temp;
            memcpy(&temp, &this->buffer[9], sizeof(char));
            
            //ROS_INFO("goal_type: %d", (unsigned int)temp);
            this->goal_type = (unsigned int)temp;
            memcpy(&this->goal_x, &this->buffer[10], sizeof(float));
            memcpy(&this->goal_y, &this->buffer[14], sizeof(float));
            this->buffer.erase(this->buffer.begin(), this->buffer.begin()+read_length);
            this->receive_message = 1;
            //ROS_INFO("Read from buffer");
            return true;
        }
        else {
            this->buffer.erase(this->buffer.begin());
            return true; // Format dismatch, drop the first byte and try again 
        }
    }
    void printData() {
        ROS_INFO_STREAM("imu_angle: " << this->imu_angle);
        ROS_INFO_STREAM("relative_angle: " << this->relative_angle);
    }
};

geometry_msgs::Twist cmd_vel;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel = *msg;
    // printf("msg_received");
    // std::cout<<cmd_vel.linear.x<<' '<<cmd_vel.linear.y<<std::endl;
}
void StatusCallback(const std_msgs::Bool::ConstPtr &msg)
{
    // cmd_vel = *msg;
    status = 1;
    // printf("msg_received");
    // std::cout<<cmd_vel.linear.x<<' '<<cmd_vel.linear.y<<std::endl;
}
// std::pair<double, double> getGoalType[]={
// {8.215254783630371, 8.674027442932129},    //0
// {8.011299133300781, 25.099763870239258},   //1
// {5.136077880859375, 13.348814010620117},   //2
// {10.739795684814453, 20.710840225219727},  //3
// {2.661466598510742, 17.89205551147461},    //4
// {13.056159973144531,  15.860123634338379}, //5
// {4.178987979888916, 20.687192916870117},   //6
// {4.229683876037598, 11.059250831604004},   //7
// {11.635642051696777, 22.893033981323242},  //8
// {13.664833068847656, 11.683853149414062},  //9
// {14.618308067321777, 5.061775207519531},   //10
// {13.056159973144531,  15.860123634338379}  //11
// };
// double angles[2]={0.0, -35.0/180.0*3.14159265};


/* The following are for testing at Jul29*/
// std::vector<std::pair<double, double> > getGoalType[]={
// {{0.3, 1.45}, {0.58, -1.7}},    //0
// {{1.82, 2.06}},   //1
// {{2.15, -2.05}},//2
// {{10.739795684814453, 20.710840225219727}},  //3
// {{3.51, -1.79}},    //4
// {{3.51, -1.80}, {3.57, -3.96}},  //5
// {{0.7, -1.6}}, //6
// {{4.229683876037598, 11.059250831604004}},   //7
// {{3.3, -5.8}},   //8
// {{0.3, 1.45}},  //9
// {{3.52, 2.08}},   //10
// {{13.056159973144531,  15.860123634338379}}  //11
// };
/*
    UC MAP    
// */
// UL MAP
// std::vector<std::pair<double, double> > getGoalType[]={
// {{10.2, -8.0}, {5.73, -8.0}},    //0
// {{7.5,-1.0}},   //1
// {{4.6,-1.0}},//2
// {{1.15,-1.0}},  //3
// {{7.2,-8.33}},    //4
// {{4.5,-6.2}},  //5
// {{1.5,-4.75}}, //6
// {{7.47,-11.5}},   //7
// {{4.6,-11.5}},   //8
// {{1.15,-11.5}},  //9
// {{4.5,-6.2},{3.0,-3.0},{7.0,-3.0},{7.0,-6.0}},//10 patrol in our base
// {{4.5,-6.2},{5.0,-10.0},{2.25,-9.4},{2.0,-7.0}},//11 patrol in enemy's base
// {{5.0,-6.0},{4.5,-6.4},{3.8,-6.0}}//12
// };

std::vector<std::pair<double, double> > getGoalType[]={
    {{10.2, -8.0}, {5.73, -8.0}},    //0
    {{7.5,-1.0}},   //1
    {{4.6,-1.0}},//2
    {{1.15,-1.0}},  //3
    {{7.2,-8.33}},    //4
    {{1.0,-1.0},{7.0,-3.0}},  //5
    {{1.5,-4.75}}, //6
    {{7.47,-11.5}},   //7
    {{4.6,-11.5}},   //8
    {{1.15,-11.5}},  //9
    {{1.0,-1.0},{7.0,-3.0}},//10 patrol in our base
    {{1.0,-1.0},{7.0,-3.0}},//11 patrol in enemy's base
    {{1.0,-1.0},{7.0,-3.0}}//12
    };
// std::vector<std::pair<double, double> > getGoalType[]={
// {{}},
// {{-2.0,0.0}},
// {{-3.0,-1.8}},
// {{}},
// {{0.0,0.0}},
// {{0.0,-2.0}}
// };
// std::vector<std::pair<double, double> > getGoalType[]={
// {{}},
// {{2.0,0.0}},
// {{8.0,2.0}},
// {{8.0,0.0}},
// {{2.5,-3.0}},
// {{0.0,0.0},{8.0,0.0}},
// {{8.0,2.0},{2.5,-3.0}}
// };

// std::vector<std::pair<double, double> > getGoalType[]={
// {{3.0, -3.0}, {1.0, -4.0}},    //0
// {{-2.0, -1.5}},   //1
// {{1.16, 0.71}},//2
// {{-2.56775765, -4.34567654567}} //3
// };
double angles[2]={0.0, 0.0};


int main(int argc, char** argv)
{
    std::string node_name = "ser2msg_decision_givepoint";
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;

    std::string serial_port;
    if (!nh.getParam("/"+node_name+"/serial_port", serial_port))
    {
        ROS_ERROR("Failed to retrieve parameter 'serial_port'");
        return -1;
    }
    float delta_time;
    if (!nh.getParam("/"+node_name+"/delta_time", delta_time))
    {
        ROS_ERROR("Failed to retrieve parameter 'delta_time'");
        return -1;
    }
    
    // 裁判系统参数读取
    std::string referee_port;
    if (!nh.getParam("/"+node_name+"/referee_port", referee_port)) {
        referee_port = "/dev/ttyUSB1";  // 默认裁判系统串口
    }
    int robot_id;
    if (!nh.getParam("/"+node_name+"/robot_id", robot_id)) {
        robot_id = 7;  // 默认红方哨兵
    }
    bool is_red_team = true;
    if (!nh.getParam("/"+node_name+"/is_red_team", is_red_team)) {
        is_red_team = true;
    }
    
    std::string virtual_frame;
    if (!nh.getParam("/"+node_name+"/virtual_frame", virtual_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'virtual_frame'");
        return -1;
    }
    std::string rotbase_frame;
    if (!nh.getParam("/"+node_name+"/rotbase_frame", rotbase_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'rotbase_frame'");
        return -1;
    }
    std::string gimbal_frame;
    if (!nh.getParam("/"+node_name+"/gimbal_frame", gimbal_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'gimbal_frame'");
        return -1;
    }
    std::string vel_topic;
    if (!nh.getParam("/"+node_name+"/vel_topic",vel_topic)){
    	ROS_ERROR("Failed to get param: %s", vel_topic.c_str());
    }
    std::string _3DLidar_frame;
    if (!nh.getParam("/"+node_name+"/_3DLidar_frame", _3DLidar_frame))
    {
        ROS_ERROR("Failed to retrieve parameter '_3DLidar_frame'");
        return -1;
    }
    // For 云台手
    double theta=0;
    if (!nh.getParam("/"+node_name+ "/theta", theta))
    {
        ROS_ERROR("Failed to retrieve parameter 'theta'");
        return -1;
    }
    double shift_x=0;
    if (!nh.getParam("/"+node_name+ "/shift_x", shift_x))
    {
        ROS_ERROR("Failed to retrieve parameter 'shift_x'");
        return -1;
    }
    double shift_y=0;
    if (!nh.getParam("/"+node_name+ "/shift_y", shift_y))
    {
        ROS_ERROR("Failed to retrieve parameter 'shift_y'");
        return -1;
    }



    // message.setTransform(virtual_frame, rotbase_frame);

    try
    {
        // Configure the serial port
        ser.setPort(serial_port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100000);
        ser.setTimeout(to);
        ser.setBytesize(serial::eightbits);
        // ser.setStopbits(serial::stopbits_one);
        ser.setParity(serial::parity_none);
        ser.setFlowcontrol(serial::flowcontrol_none);

        // Open the serial port
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open the serial port");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized");
    }
    else
    {
        return -1;
    }
    
    // 初始化裁判系统解析器
    referee_parser = new RefereeParser(nh, robot_id, is_red_team);
    
    // 尝试打开裁判系统串口
    try {
        referee_ser.setPort(referee_port);
        referee_ser.setBaudrate(REFEREE_BAUDRATE);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        referee_ser.setTimeout(to);
        referee_ser.setBytesize(serial::eightbits);
        referee_ser.setParity(serial::parity_none);
        referee_ser.setFlowcontrol(serial::flowcontrol_none);
        referee_ser.open();
        referee_connected = true;
        ROS_INFO_STREAM("Referee system connected on " << referee_port);
    } catch (serial::IOException& e) {
        ROS_WARN_STREAM("Unable to open referee serial port: " << e.what());
        ROS_WARN("Running without referee system data");
    }

    ros::Subscriber sub = nh.subscribe(vel_topic, 1000, cmdVelCallback);
    ros::Subscriber sub_status = nh.subscribe("/dstar_status", 1000, StatusCallback);
    ros::Publisher clicked_point_pub=nh.advertise<geometry_msgs::PointStamped>("clicked_point",1);

    tf2_ros::TransformBroadcaster tfb;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    tf2::Quaternion q1;
    transformRotbaseToVirtual.header.frame_id = rotbase_frame;
    transformRotbaseToVirtual.child_frame_id = virtual_frame;
    transformRotbaseToVirtual.transform.translation.x = 0.0;
    transformRotbaseToVirtual.transform.translation.y = 0.0;
    transformRotbaseToVirtual.transform.translation.z = -0.1;

    
    tf2::Quaternion q2;
    transformGimbalToRotbase.header.frame_id = gimbal_frame;
    transformGimbalToRotbase.child_frame_id = rotbase_frame;
    transformGimbalToRotbase.transform.translation.x = 0.0;
    transformGimbalToRotbase.transform.translation.y = 0.0;
    transformGimbalToRotbase.transform.translation.z = -0.3;

    Message message;
    uint8_t byte;
    uint8_t buffer_send[read_length];
    buffer_send[0] = 0x4A; // SOF
    ros::Rate rate = ros::Rate(1/delta_time);

    tf2::Transform gimbalframe;
    tf2::Transform rotbaseframe;
    tf2::Transform virtualframe;
    tf2::Transform location;

    loc.header.frame_id = "map";
    loc.child_frame_id = virtual_frame;

    int last_goal = -1, current_goal = -1, current_index = 0, cnt = 0;
    double last_goal_x, last_goal_y;
    bool first_arrive_flag = 0, cnt_flag = 0;
    
    while(ros::ok()){

        // 读取裁判系统数据
        if (referee_connected && referee_ser.available()) {
            std::vector<uint8_t> referee_data;
            size_t bytes_available = referee_ser.available();
            referee_data.resize(bytes_available);
            referee_ser.read(referee_data.data(), bytes_available);
            if (referee_parser) {
                referee_parser->receiveData(referee_data.data(), bytes_available);
            }
        }

        // Read data from the serial port
        // ROS_INFO("Reading data from the serial port");

        size_t bytes_available = ser.available();

        while (ser.available() && ros::ok())
        {
            ser.read(&byte,1);
            message.buffer.push_back(byte);
            //ROS_INFO("%d", message.buffer.size());
        }
        while (message.readFromBuffer() && ros::ok());
            // message.printData();
        //Calculate virtual frame
        {
            q1.setRPY(0,0,-message.imu_angle);
            transformRotbaseToVirtual.transform.rotation.x = q1.x();
            transformRotbaseToVirtual.transform.rotation.y = q1.y();
            transformRotbaseToVirtual.transform.rotation.z = q1.z();
            transformRotbaseToVirtual.transform.rotation.w = q1.w();
            transformRotbaseToVirtual.header.stamp = ros::Time::now();
            // tfb.sendTransform(transformRotbaseToVirtual);

            q2.setRPY(0,0,-message.relative_angle);
            transformGimbalToRotbase.transform.rotation.x = q2.x();
            transformGimbalToRotbase.transform.rotation.y = q2.y();
            transformGimbalToRotbase.transform.rotation.z = q2.z();
            transformGimbalToRotbase.transform.rotation.w = q2.w();
            transformGimbalToRotbase.header.stamp = ros::Time::now();
            // tfb.sendTransform(transformGimbalToRotbase);
            

            try{
                transformMapToGimbal = tfBuffer.lookupTransform("map", "gimbal_frame",ros::Time(0),ros::Duration(5.0));
            }
            catch(tf2::TransformException &ex){
                ROS_WARN("%s",ex.what());
            }
            tf2::fromMsg(transformMapToGimbal.transform, gimbalframe);
            tf2::fromMsg(transformGimbalToRotbase.transform,rotbaseframe);
            tf2::fromMsg(transformRotbaseToVirtual.transform,virtualframe);
            // location = gimbalframe;
            // // * rotbaseframe * virtualframe;
            location = gimbalframe * rotbaseframe * virtualframe;
            loc.transform = tf2::toMsg(location);
            loc.header.stamp = ros::Time::now();
            tfb.sendTransform(loc);
            
        }
        // ROS_INFO("continue 1");
        //send goal
        {
            ROS_INFO("goal_type: %d",message.goal_type);
            if(message.goal_type != current_goal || (last_goal == 0xF0 && (message.goal_x != last_goal_x || message.goal_y != last_goal_y))){
                last_goal = current_goal;
                current_goal = message.goal_type;
                last_goal_x = message.goal_x;
                last_goal_y = message.goal_y;
                current_index = 0;
                cnt = cnt_flag = 0;
                first_arrive_flag = 0;
            }
            // ROS_INFO("continue 1 1");
            if(message.goal_type != 13){
                geometry_msgs::PointStamped clicked_point;
                clicked_point.header.frame_id="map";
                clicked_point.header.stamp=ros::Time::now();
                // ROS_INFO("continue 1 2");
                if(message.goal_type != 0xF0){
                    if(status){
                        cnt_flag = 1;
                    } 
                    if(cnt_flag){
                        cnt++;
                    }
                    if (cnt == int(1 / delta_time)&&(message.goal_type == 11 || message.goal_type == 10 || message.goal_type == 12 || message.goal_type == 5)){
                        cnt = cnt_flag = 0;
                        current_index = (current_index + 1) %  getGoalType[message.goal_type].size();
                    }
                    // if(cnt == int(8 / delta_time) && (message.goal_type==5)){
                    //     cnt = cnt_flag = 0;
                    //     current_index = (current_index + 1) %  getGoalType[message.goal_type].size();
                    // }
                    // if(cnt == int(1 / delta_time) && message.goal_type == 0){
                    //     cnt = cnt_flag = 0;
                    //     current_index = (current_index + 1) %  getGoalType[message.goal_type].size();
                    // }
                    clicked_point.point.x=getGoalType[message.goal_type][current_index].first;
                    clicked_point.point.y=getGoalType[message.goal_type][current_index].second;
                    clicked_point.point.z=0;
                }
                else{
                    clicked_point.point.x=cos(theta)*message.goal_x - sin(theta)*message.goal_y + shift_x;
                    clicked_point.point.y=sin(theta)*message.goal_x + cos(theta)*message.goal_y + shift_y;
                    ROS_INFO("shift_x: %f, shift_y: %f", shift_x, shift_y);
                    ROS_INFO("x: %.3f,y: %.3f", message.goal_x, message.goal_y);
                    clicked_point.point.z=0;
                }
                //clicked_point_pub.publish(clicked_point);
            }
        }
        // ROS_INFO("continue 2");
        // Write data to serial port
        // Linear velocities x
        {   
            FloatToByte linear_x;
            linear_x.f = cmd_vel.linear.x;
            std::copy(std::begin(linear_x.bytes),std::end(linear_x.bytes),&buffer_send[1]);
        }
        // Linear velocities y
        {
            FloatToByte linear_y;
            linear_y.f = -cmd_vel.linear.y;
            std::copy(std::begin(linear_y.bytes),std::end(linear_y.bytes),&buffer_send[5]);
        }
        // ROS_INFO("continue 3");
        // Omega angle w
        {
            FloatToByte omega;
            // int index = 0;
            // if(current_goal == 2 || last_goal == 2){
            //     index = 1;
            // }
            // auto tf_rot_from_map_to_virtual = loc;
            // tf_rot_from_map_to_virtual.transform.translation.x = 0;
            // tf_rot_from_map_to_virtual.transform.translation.y = 0;
            // tf_rot_from_map_to_virtual.transform.translation.z = 0;
            // geometry_msgs::PointStamped source_point;
            // source_point.header.frame_id = "virtual_frame";
            // source_point.header.stamp = ros::Time(0);
            // source_point.point.x = cos(angles[index]);
            // source_point.point.y = sin(angles[index]);
            // ROS_INFO("x: %lf y: %lf angle: %lf index: %d", source_point.point.x, source_point.point.y, angles[index], index);
            // source_point.point.z = 0;
            // geometry_msgs::PointStamped target_point;
            // tf2::doTransform(source_point, target_point, tf_rot_from_map_to_virtual);
            // double rx = target_point.point.x;
            // double ry = target_point.point.y;
            // double angle = atan2(ry,rx);
            // omega.f = -angle;
            omega.f = 0;
            std::copy(std::begin(omega.bytes),std::end(omega.bytes),&buffer_send[9]);
        }
        //received message
        {
            ByteToByte received_message;
            received_message.f = message.goal_type == 0xF0;
            std::copy(std::begin(received_message.bytes),std::end(received_message.bytes),&buffer_send[13]);
            // message.receive_message = 0;
        }
        // ROS_INFO("continue 4");
        //whether arrived
        {
            ByteToByte arrived;//0 move, 1 arrive
            first_arrive_flag |= status;
            arrived.f = first_arrive_flag | (message.goal_type == 13);
            status = 0;
            std::copy(std::begin(arrived.bytes),std::end(arrived.bytes),&buffer_send[14]);
        }
        // Write to the serial port
        size_t bytes_written = ser.write(buffer_send,write_length);
        if (bytes_written < write_length){
            ROS_ERROR("Failed to write all bytes to the serial port");
        }
        //ROS_INFO("relative angle: %f imu_angle: %f", message.relative_angle, message.imu_angle);
        // ROS_INFO("%f %f %f", linear_x.f, linear_y.f, omega.f);
        ros::spinOnce(); 
        rate.sleep();
    }
    
    // 清理裁判系统资源
    if (referee_parser) {
        delete referee_parser;
        referee_parser = nullptr;
    }
    
    return 0;
}