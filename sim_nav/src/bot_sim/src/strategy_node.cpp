/*
决策模块
通信：
  订阅裁判系统相关话题（占位，后续可替换为 Referee_Task 的真实桥接）
  订阅导航到达状态话题（复用现有语义）
  发布目标点话题（clicked_point）
[TODO] 
- 后续可接真实视觉数据
- 修改上下位机通讯逻辑，只负责运动控制的收发
- 在决策判断是否到达感觉很别扭，是否放在上下位机通讯模块更合适？
*/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

namespace
{
constexpr int kDefaultTickHz = 20;

std::string toUpper(std::string s)
{
  for (auto& c : s)
  {
    c = static_cast<char>(::toupper(c));
  }
  return s;
}

}  // namespace

// ---------------------------
// Shared state (ROS callbacks)
// ---------------------------
struct RefereeState
{
  int game_progress = 0;  // 0: not start, 1: in progress, 2: end (convention)
  int remain_hp = 400;
  int bullet_remain = 999;
  int friendly_score = 0;
  int enemy_score = 0;
};

struct NavigationState
{
  bool arrived = false;
};

// ---------------------------
// BT Nodes: Update blackboard
// ---------------------------
class UpdateRefereeBB : public BT::SyncActionNode
{
public:
  UpdateRefereeBB(const std::string& name, const BT::NodeConfiguration& config, const RefereeState* state)
    : BT::SyncActionNode(name, config), state_(state)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    bb->set("ref.game_progress", state_->game_progress);
    bb->set("ref.remain_hp", state_->remain_hp);
    bb->set("ref.bullet_remain", state_->bullet_remain);
    bb->set("ref.friendly_score", state_->friendly_score);
    bb->set("ref.enemy_score", state_->enemy_score);
    return BT::NodeStatus::SUCCESS;
  }

private:
  const RefereeState* state_;
};

class UpdateNavigationBB : public BT::SyncActionNode
{
public:
  UpdateNavigationBB(const std::string& name, const BT::NodeConfiguration& config, const NavigationState* state)
    : BT::SyncActionNode(name, config), state_(state)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    config().blackboard->set("nav.arrived", state_->arrived);
    return BT::NodeStatus::SUCCESS;
  }

private:
  const NavigationState* state_;
};

class UpdateVisionBB : public BT::SyncActionNode
{
public:
  UpdateVisionBB(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // V1: explicitly弃用视觉模块。这里保持节点存在，但不写入任何视觉字段。[TODO] 后续可接真实视觉数据
    return BT::NodeStatus::SUCCESS;
  }
};

class UpdateTimersBB : public BT::SyncActionNode
{
public:
  UpdateTimersBB(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // 预留：[TODO] 后续可把 Strategy_Task.c 里的计数器/超时机制迁移到这里
    return BT::NodeStatus::SUCCESS;
  }
};

class UpdateDerivedFlags : public BT::SyncActionNode
{
public:
  UpdateDerivedFlags(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("danger_hp", 100, "HP threshold: below which is considered 'in danger'"),
      BT::InputPort<int>("sufficient_bullet", 10, "Bullet threshold: below which is considered insufficient"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    const int remain_hp = bb->get<int>("ref.remain_hp");
    const int bullet_remain = bb->get<int>("ref.bullet_remain");

    int danger_hp = 100;
    (void)getInput("danger_hp", danger_hp);

    int sufficient_bullet = 10;
    (void)getInput("sufficient_bullet", sufficient_bullet);

    const bool is_dead = (remain_hp <= 0);
    const bool is_in_danger = (remain_hp > 0 && remain_hp < danger_hp);
    const bool bullet_sufficient = (bullet_remain >= sufficient_bullet);

    bb->set("is_dead", is_dead);
    bb->set("is_in_danger", is_in_danger);
    bb->set("bullet_sufficient", bullet_sufficient);

    // V1: aggressive / central_occupiable 先作为占位，后续接裁判/受击统计
    try
    {
      (void)bb->get<bool>("is_aggressive");
    }
    catch (...)
    {
      bb->set("is_aggressive", false);
    }
    try
    {
      (void)bb->get<bool>("central_occupiable");
    }
    catch (...)
    {
      bb->set("central_occupiable", false);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

// ---------------------------
// BT Nodes: Conditions
// ---------------------------
class GameNotStartEnd : public BT::ConditionNode
{
public:
  GameNotStartEnd(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const int gp = config().blackboard->get<int>("ref.game_progress");
    const bool not_start_or_end = (gp == 0 || gp == 2);
    return not_start_or_end ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsSentryDead : public BT::ConditionNode
{
public:
  IsSentryDead(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool is_dead = config().blackboard->get<bool>("is_dead");
    return is_dead ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsSentryInDanger : public BT::ConditionNode
{
public:
  IsSentryInDanger(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool in_danger = config().blackboard->get<bool>("is_in_danger");
    return in_danger ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class NotBulletSufficient : public BT::ConditionNode
{
public:
  NotBulletSufficient(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool bullet_sufficient = config().blackboard->get<bool>("bullet_sufficient");
    return (!bullet_sufficient) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsArrived : public BT::ConditionNode
{
public:
  IsArrived(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool arrived = config().blackboard->get<bool>("nav.arrived");
    return arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class AggressiveAdvantage : public BT::ConditionNode
{
public:
  AggressiveAdvantage(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("threshold", 50, "friendly_score-enemy_score 阈值"),
    };
  }

  BT::NodeStatus tick() override
  {
    int threshold = 50;
    (void)getInput("threshold", threshold);

    const bool aggressive = config().blackboard->get<bool>("is_aggressive");
    const int friendly = config().blackboard->get<int>("ref.friendly_score");
    const int enemy = config().blackboard->get<int>("ref.enemy_score");

    const bool ok = aggressive && (friendly - enemy >= threshold);
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class CentralOccupiable : public BT::ConditionNode
{
public:
  CentralOccupiable(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool occupiable = config().blackboard->get<bool>("central_occupiable");
    return occupiable ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsAction : public BT::ConditionNode
{
public:
  IsAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("value", "INIT", "Action name")};
  }

  BT::NodeStatus tick() override
  {
    std::string value;
    if (!getInput("value", value))
    {
      return BT::NodeStatus::FAILURE;
    }

    std::string action;
    try
    {
      action = config().blackboard->get<std::string>("action");
    }
    catch (...)
    {
      return BT::NodeStatus::FAILURE;
    }

    return (toUpper(action) == toUpper(value)) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ---------------------------
// BT Nodes: Actions
// ---------------------------
class SetAction : public BT::SyncActionNode
{
public:
  SetAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("action")}; }

  BT::NodeStatus tick() override
  {
    std::string action;
    if (!getInput("action", action))
    {
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set("action", toUpper(action));
    return BT::NodeStatus::SUCCESS;
  }
};

class SetMotion : public BT::SyncActionNode
{
public:
  SetMotion(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("motion")}; }

  BT::NodeStatus tick() override
  {
    std::string motion;
    if (!getInput("motion", motion))
    {
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set("motion", toUpper(motion));
    return BT::NodeStatus::SUCCESS;
  }
};

class ClearGoal : public BT::SyncActionNode
{
public:
  ClearGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    bb->set("goal.valid", false);
    return BT::NodeStatus::SUCCESS;
  }
};

//[TODO]: 这里的目标点是从参数服务器读取的，
class SetGoalFromParams : public BT::SyncActionNode
{
public:
  SetGoalFromParams(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle* nh)
    : BT::SyncActionNode(name, config), nh_(nh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ns", "push", "parameter namespace: e.g. push/occupy/supply"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string ns;
    if (!getInput("ns", ns))
    {
      return BT::NodeStatus::FAILURE;
    }

    const std::string base = std::string("goals/") + ns;

    double x = 0.0, y = 0.0;
    (void)nh_->param(base + "/x", x, 0.0);
    (void)nh_->param(base + "/y", y, 0.0);

    geometry_msgs::PointStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.point.x = x;
    goal.point.y = y;
    goal.point.z = 0.0;

    auto bb = config().blackboard;
    bb->set("goal.point", goal);
    bb->set("goal.valid", true);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle* nh_;
};

class PublishGoalPoint : public BT::SyncActionNode
{
public:
  PublishGoalPoint(const std::string& name,
                   const BT::NodeConfiguration& config,
                   ros::Publisher* pub,
                   bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), pub_(pub), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "clicked_point", "target topic"),
    };
  }

  BT::NodeStatus tick() override
  {
    (void)getInput("topic", last_topic_);  // retained only for XML readability

    auto bb = config().blackboard;
    const bool valid = bb->get<bool>("goal.valid");
    if (!valid)
    {
      return BT::NodeStatus::SUCCESS;
    }

    const auto goal = bb->get<geometry_msgs::PointStamped>("goal.point");

    if (*publish_on_change_only_)
    {
      // crude de-dup: compare x/y only
      bool have_last = false;
      double last_x = 0.0, last_y = 0.0;
      try
      {
        last_x = bb->get<double>("goal.last_x");
        last_y = bb->get<double>("goal.last_y");
        have_last = true;
      }
      catch (...)
      {
        have_last = false;
      }

      if (have_last && goal.point.x == last_x && goal.point.y == last_y)
      {
        return BT::NodeStatus::SUCCESS;
      }
      bb->set("goal.last_x", goal.point.x);
      bb->set("goal.last_y", goal.point.y);
    }

    pub_->publish(goal);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::Publisher* pub_;
  bool* publish_on_change_only_;
  std::string last_topic_;
};

// ---------------------------
// Main
// ---------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "strategy_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  RefereeState ref;
  NavigationState nav;

  // Referee-like inputs (占位话题；[TODO] 后续可替换为 Referee_Task 的真实桥接)
  auto sub_game_progress = nh.subscribe<std_msgs::Int32>("/referee/game_progress", 1, [&](const std_msgs::Int32::ConstPtr& msg) {
    ref.game_progress = msg->data;
  });
  auto sub_remain_hp = nh.subscribe<std_msgs::Int32>("/referee/remain_hp", 1, [&](const std_msgs::Int32::ConstPtr& msg) {
    ref.remain_hp = msg->data;
  });
  auto sub_bullet = nh.subscribe<std_msgs::Int32>("/referee/bullet_remain", 1, [&](const std_msgs::Int32::ConstPtr& msg) {
    ref.bullet_remain = msg->data;
  });
  auto sub_friendly_score = nh.subscribe<std_msgs::Int32>("/referee/friendly_score", 1, [&](const std_msgs::Int32::ConstPtr& msg) {
    ref.friendly_score = msg->data;
  });
  auto sub_enemy_score = nh.subscribe<std_msgs::Int32>("/referee/enemy_score", 1, [&](const std_msgs::Int32::ConstPtr& msg) {
    ref.enemy_score = msg->data;
  });

  // Navigation arrived (复用现有语义)
  auto sub_arrived = nh.subscribe<std_msgs::Bool>("/dstar_status", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    nav.arrived = msg->data;
  });

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 1);

  int tick_hz = kDefaultTickHz;
  pnh.param("tick_hz", tick_hz, tick_hz);

  bool publish_on_change_only = true;
  pnh.param("publish_on_change_only", publish_on_change_only, publish_on_change_only);

  BT::BehaviorTreeFactory factory;

  // Register custom nodes.
  factory.registerBuilder<UpdateRefereeBB>(
    "UpdateRefereeBB", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<UpdateRefereeBB>(name, config, &ref);
    });

  factory.registerBuilder<UpdateNavigationBB>(
    "UpdateNavigationBB", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<UpdateNavigationBB>(name, config, &nav);
    });

  factory.registerNodeType<UpdateVisionBB>("UpdateVisionBB");
  factory.registerNodeType<UpdateTimersBB>("UpdateTimersBB");
  factory.registerNodeType<UpdateDerivedFlags>("UpdateDerivedFlags");

  factory.registerNodeType<GameNotStartEnd>("GameNotStartEnd");
  factory.registerNodeType<IsSentryDead>("IsSentryDead");
  factory.registerNodeType<IsSentryInDanger>("IsSentryInDanger");
  factory.registerNodeType<NotBulletSufficient>("NotBulletSufficient");
  factory.registerNodeType<IsArrived>("IsArrived");
  factory.registerNodeType<AggressiveAdvantage>("AggressiveAdvantage");
  factory.registerNodeType<CentralOccupiable>("CentralOccupiable");
  factory.registerNodeType<IsAction>("IsAction");

  factory.registerNodeType<SetAction>("SetAction");
  factory.registerNodeType<SetMotion>("SetMotion");
  factory.registerNodeType<ClearGoal>("ClearGoal");

  factory.registerBuilder<SetGoalFromParams>(
    "SetGoalFromParams", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SetGoalFromParams>(name, config, &nh);
    });

  factory.registerBuilder<PublishGoalPoint>(
    "PublishGoalPoint", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<PublishGoalPoint>(name, config, &goal_pub, &publish_on_change_only);
    });

  auto blackboard = BT::Blackboard::create();

  // Blackboard defaults (can be overridden with params below)
  int danger_hp = 100;
  int sufficient_bullet = 10;
  pnh.param("danger_hp", danger_hp, danger_hp);
  pnh.param("sufficient_bullet", sufficient_bullet, sufficient_bullet);
  blackboard->set("danger_hp", danger_hp);
  blackboard->set("sufficient_bullet", sufficient_bullet);

  // Default action
  blackboard->set("action", std::string("INIT"));
  blackboard->set("motion", std::string("STAY_IN_PLACE"));
  blackboard->set("goal.valid", false);
  blackboard->set("is_aggressive", false);
  blackboard->set("central_occupiable", false);

  std::string bt_xml_path;
  pnh.param<std::string>("bt_xml", bt_xml_path, std::string(""));
  if (bt_xml_path.empty())
  {
    bt_xml_path = ros::package::getPath("bot_sim") + "/config/strategy_tree.xml";
  }

  std::ifstream xml_file(bt_xml_path);
  if (!xml_file.is_open())
  {
    ROS_FATAL_STREAM("Failed to open bt_xml file: " << bt_xml_path);
    return 1;
  }
  std::stringstream xml_buffer;
  xml_buffer << xml_file.rdbuf();
  const std::string xml_text = xml_buffer.str();

  BT::Tree tree = factory.createTreeFromText(xml_text, blackboard);

  ros::Rate rate(std::max(1, tick_hz));
  while (ros::ok())
  {
    ros::spinOnce();
    tree.tickRoot();
    rate.sleep();
  }

  return 0;
}
