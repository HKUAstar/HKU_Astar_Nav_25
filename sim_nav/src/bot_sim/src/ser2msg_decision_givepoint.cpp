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
//新增头文件
#include <std_msgs/Int32.h>      
#include <std_msgs/UInt8.h>  
#include <std_msgs/Float32.h>
#include <vector>

const double PI = 3.14159265358979323846;

serial::Serial ser;

const int write_length = 15;
const int read_length = 19;

// 根据referee_task中的函数得到
const int referee_data_length = 62;
const int total_read_length = read_length + referee_data_length;


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
     //新增转发裁判系统数据
    uint8_t game_progress = 0;            
    uint16_t remain_hp = 400;           
    uint16_t bullet_remain = 999;        
    
    
    uint8_t robot_color = 0;             // 己方颜色 1:红方 2:蓝方
    uint8_t red_dead = 0;                
    uint8_t blue_dead = 0;               
    uint16_t occupy_status = 0;          
    
    // 分数计算相关
    int friendly_score = 0;              
    int enemy_score = 0;                 
    
    //击杀
    uint8_t previous_red_dead = 0;
    uint8_t previous_blue_dead = 0;
    uint16_t check_occupation_cnt = 0;   // 中央占领检测计数器

    bool readFromBuffer() {
        if (this->buffer.size() < total_read_length) {
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
            
              
            // 获取裁判数据
            if (this->buffer.size() >= total_read_length) {
                uint8_t* referee_data = &this->buffer[19];
                
                this->robot_color = referee_data[1];           
                this->game_progress = referee_data[2];         
                this->remain_hp = (referee_data[6] << 8) | referee_data[5];  
                
                // 子弹数量（data_unpack）
                this->bullet_remain = (referee_data[53] << 8) | referee_data[52];
                
                // 占领状态
                uint32_t event_data = (referee_data[51] << 24) | (referee_data[50] << 16) | 
                                      (referee_data[49] << 8) | referee_data[48];
                this->occupy_status = (event_data >> 23) & 0x03;  // bit23-24: 中心增益点占领状态
                
                // 保存上一次的死亡状态用于检测击杀
                this->previous_red_dead = this->red_dead;
                this->previous_blue_dead = this->blue_dead;
                
                updateDeathStatus(referee_data);
                
                //更新分数
                updateScore();
            }
            
            this->buffer.erase(this->buffer.begin(), this->buffer.begin()+total_read_length);
            this->receive_message = 1;
            //ROS_INFO("Read from buffer");
            return true;
        }
        else {
            this->buffer.erase(this->buffer.begin());
            return true; // Format dismatch, drop the first byte and try again 
        }
    }
        // 更新死亡状态（根据血量判断）
    void updateDeathStatus(uint8_t* data) {
        // 假设血量数据在第16-39字节（根据data_unpack）
        // red_1_hp: 16-17, red_2_hp: 18-19, red_3_hp: 20-21, red_4_hp: 22-23
        // blue_1_hp: 24-25, blue_2_hp: 26-27, blue_3_hp: 28-29, blue_4_hp: 30-31
        
        this->red_dead = 0;
        this->blue_dead = 0;
        
        // 红方机器人死亡状态
        for (int i = 0; i < 4; i++) {
            uint16_t hp = (data[17 + i*2] << 8) | data[16 + i*2];
            if (hp == 0) {
                this->red_dead |= (1 << i);
            }
        }
        
        // 蓝方机器人死亡状态
        for (int i = 0; i < 4; i++) {
            uint16_t hp = (data[25 + i*2] << 8) | data[24 + i*2];
            if (hp == 0) {
                this->blue_dead |= (1 << i);
            }
        }
    }

    // 更新分数
    void updateScore() {
        if (this->game_progress != 4) {  // 不在比赛中
            return;      
        }
        
        // 1. 占领中央增益点所得分数（每20次检测加1分）
        this->check_occupation_cnt++;
        if (this->check_occupation_cnt >= 20) {  // 相当于1秒（假设50ms一次）
            this->check_occupation_cnt = 0;
            
            if (this->occupy_status == 2) {  // 被对方占领
                this->enemy_score++;
            }
            else if (this->occupy_status == 1) {  // 被己方占领
                this->friendly_score++;
            }
        }
        
        // 2. 击杀所得分数（检测新死亡）
        for (int index = 0; index < 4; index++) {  // 4个机器人
            bool previous_blue_dead_bit = (this->previous_blue_dead >> index) & 0x01;
            bool current_blue_dead_bit = (this->blue_dead >> index) & 0x01;
           
            if (previous_blue_dead_bit == 0 && current_blue_dead_bit == 1) {
                // 蓝方机器人新死亡
                if (this->robot_color == 2) {  // 己方是蓝色
                    this->enemy_score += 20;
                }
                else {  // 己方是红色
                    this->friendly_score += 20;
                }
            }
            
            bool previous_red_dead_bit = (this->previous_red_dead >> index) & 0x01;
            bool current_red_dead_bit = (this->red_dead >> index) & 0x01;
            
            if (previous_red_dead_bit == 0 && current_red_dead_bit == 1) {
                // 红方机器人新死亡
                if (this->robot_color == 2) {  // 己方是蓝色
                    this->friendly_score += 20;
                }
                else {  // 己方是红色
                    this->enemy_score += 20;
                }
            }
        }
    }
    
    void printData() {
        ROS_INFO_STREAM("Game Progress: " << (int)this->game_progress 
                       << ", HP: " << this->remain_hp 
                       << ", Bullet: " << this->bullet_remain
                       << ", Score: " << this->friendly_score << "-" << this->enemy_score);
    }

    bool isCentralOccupiable() const {
        // occupy_status: bit23-24 of event_data (0: none, 1/2: occupied, 3: other/contested)
        // Simplest interpretation: "occupiable" when currently unoccupied during the match.
        return (this->game_progress == 4) && (this->occupy_status == 0);
    }
};

            
    // void printData() {
    //     ROS_INFO_STREAM("imu_angle: " << this->imu_angle);
    //     ROS_INFO_STREAM("relative_angle: " << this->relative_angle);
    // }


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
    // std::string vel_topic = "cmd_vel";
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

    ros::Subscriber sub = nh.subscribe(vel_topic, 1000, cmdVelCallback);
    ros::Subscriber sub_status = nh.subscribe("/dstar_status", 1000, StatusCallback);
    ros::Publisher clicked_point_pub=nh.advertise<geometry_msgs::PointStamped>("clicked_point",1);
    // 新增topic
    ros::Publisher game_progress_pub = nh.advertise<std_msgs::Int32>("/referee/game_progress", 1);
    ros::Publisher robot_hp_pub = nh.advertise<std_msgs::Int32>("/referee/remain_hp", 1);
    ros::Publisher bullet_remain_pub = nh.advertise<std_msgs::Int32>("/referee/bullet_remain", 1);
    ros::Publisher friendly_score_pub = nh.advertise<std_msgs::Int32>("/referee/friendly_score", 1);
    ros::Publisher enemy_score_pub = nh.advertise<std_msgs::Int32>("/referee/enemy_score", 1);
    
    ros::Publisher central_occupiable_pub = nh.advertise<std_msgs::Bool>("/referee/central_occupiable", 1);
    ros::Publisher aggressive_pub = nh.advertise<std_msgs::Bool>("/referee/is_aggressive", 1);
    // 运动状态发布给寻路
    ros::Publisher imu_angle_pub = nh.advertise<std_msgs::Float32>("/robot/imu_angle", 1);
    ros::Publisher relative_angle_pub = nh.advertise<std_msgs::Float32>("/robot/relative_angle", 1);
    
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
    uint8_t buffer_send[write_length];
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
        // 发布裁判数据到ROS话题
        if (message.receive_message) {
            std_msgs::Int32 game_progress_msg;
            game_progress_msg.data = message.game_progress;
            game_progress_pub.publish(game_progress_msg);
            
            std_msgs::Int32 hp_msg;
            hp_msg.data = message.remain_hp;
            robot_hp_pub.publish(hp_msg);
            
            std_msgs::Int32 bullet_msg;
            bullet_msg.data = message.bullet_remain;
            bullet_remain_pub.publish(bullet_msg);
            
            std_msgs::Int32 friendly_score_msg;
            friendly_score_msg.data = message.friendly_score;
            friendly_score_pub.publish(friendly_score_msg);
            
            std_msgs::Int32 enemy_score_msg;
            enemy_score_msg.data = message.enemy_score;
            enemy_score_pub.publish(enemy_score_msg);
         
            std_msgs::Bool central_occupiable_msg;
            central_occupiable_msg.data = message.isCentralOccupiable();
            central_occupiable_pub.publish(central_occupiable_msg);
        
            
            // 运动状态转发给寻路模块
            std_msgs::Float32 imu_angle_msg;
            imu_angle_msg.data = message.imu_angle;
            imu_angle_pub.publish(imu_angle_msg);
            
            std_msgs::Float32 relative_angle_msg;
            relative_angle_msg.data = message.relative_angle;
            relative_angle_pub.publish(relative_angle_msg);
            message.receive_message = 0;
        }
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
        // Omega angle w
        {
            FloatToByte omega;
            omega.f = 0;
            std::copy(std::begin(omega.bytes),std::end(omega.bytes),&buffer_send[9]);
        }
        //received message
        {
            ByteToByte received_message;
            received_message.f = message.goal_type == 0xF0;
            std::copy(std::begin(received_message.bytes),std::end(received_message.bytes),&buffer_send[13]);
        }
        //whether arrived
        {
            ByteToByte arrived;
            first_arrive_flag |= status;
            arrived.f = first_arrive_flag | (message.goal_type == 13);
            status = 0;
            std::copy(std::begin(arrived.bytes),std::end(arrived.bytes),&buffer_send[14]);
        }
        // 发送给下位机
        size_t bytes_written = ser.write(buffer_send,write_length);
        if (bytes_written < write_length){
            ROS_ERROR("Failed to write all bytes to the serial port");
        } 
        ros::spinOnce(); 
        rate.sleep();
    }
    return 0;
}