### V1.1
给出行为树的结构草图（XML）与黑板字段表
#### 行为树结构草图（XML）

```xml
<root BTCPP_format="4">
  <!-- 主树：每次 tick 都先更新黑板，再做决策，再执行动作 -->
  <BehaviorTree ID="Main">
    <ReactiveSequence name="MainLoop">
      <!-- 这些节点一般不做“决策”，只把 ROS 回调/缓存数据同步到黑板 -->
      <Sequence name="UpdateBlackboard">
        <UpdateRefereeBB   name="UpdateRefereeBB" />
        <UpdateNavigationBB name="UpdateNavigationBB" />
        <UpdateVisionBB    name="UpdateVisionBB" />
        <UpdateTimersBB    name="UpdateTimersBB" />
        <UpdateDerivedFlags name="UpdateDerivedFlags" />
        <!-- DerivedFlags 里把 dead/in_danger/bullet_sufficient/aggressive 等置位 -->
      </Sequence>

      <!-- 决策：按优先级选择当前 action -->
      <ReactiveFallback name="DecideAction">
        <!-- 1) 比赛未开始/已结束：回到 INIT -->
        <Sequence name="ToInitWhenNotInGame">
          <GameNotStartEnd />
          <SetAction action="INIT" />
        </Sequence>

        <!-- 2) 死亡：进入复活流程 -->
        <Sequence name="ToRespawnWhenDead">
          <IsSentryDead />
          <SetAction action="RESPAWN" />
        </Sequence>

        <!-- 3) 危险：回补给 -->
        <Sequence name="ToSupplyWhenInDanger">
          <IsSentryInDanger />
          <SetAction action="SUPPLY" />
        </Sequence>

        <!-- 4) 需要去占点的情况（弹丸不足/优势明显/中央可占领 等）：OCCUPY -->
        <Sequence name="ToOccupyWhenNeeded">
          <Fallback>
            <NotBulletSufficient />          <!-- 弹丸不足 -->
            <AggressiveAdvantage threshold="50" /> <!-- is_aggressive && (friendly-enemy)>=50 -->
            <CentralOccupiable />            <!-- 中央可占领 -->
          </Fallback>
          <SetAction action="OCCUPY" />
        </Sequence>

        <!-- 5) 默认：推进 PUSH（或你希望的 SIDEATTACK） -->
        <SetAction action="PUSH" />
      </ReactiveFallback>

      <!-- 执行：根据 action 输出目标/模式（并发布 clicked_point 等） -->
      <SwitchAction name="ExecuteAction" action="{action}">
        <Case value="INIT">
          <SubTree ID="DoInit"/>
        </Case>
        <Case value="PUSH">
          <SubTree ID="DoPush"/>
        </Case>
        <Case value="SIDEATTACK">
          <SubTree ID="DoSideAttack"/>
        </Case>
        <Case value="OCCUPY">
          <SubTree ID="DoOccupy"/>
        </Case>
        <Case value="SUPPLY">
          <SubTree ID="DoSupply"/>
        </Case>
        <Case value="RESPAWN">
          <SubTree ID="DoRespawn"/>
        </Case>
        <Default>
          <Failure/>
        </Default>
      </SwitchAction>
    </ReactiveSequence>
  </BehaviorTree>

  <!-- 各 action 的“输出子树”示例：通常就是选目标 + 发布目标 -->
  <BehaviorTree ID="DoInit">
    <Sequence>
      <SetMotion motion="STAY_IN_PLACE"/>
      <ClearGoal/>
      <PublishStrategyStatus/>  <!-- 可选：发布当前 action/motion 便于调试 -->
      <Success/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="DoPush">
    <ReactiveSequence>
      <SetMotion motion="PUSH"/>
      <SelectGoalForPush goal_id="{goal_id}" goal_point="{goal_point}"/>
      <PublishGoalPoint topic="clicked_point" point="{goal_point}"/>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="DoSideAttack">
    <ReactiveSequence>
      <SetMotion motion="SIDEATTACK"/>
      <SelectGoalForSideAttack goal_id="{goal_id}" goal_point="{goal_point}"/>
      <PublishGoalPoint topic="clicked_point" point="{goal_point}"/>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="DoOccupy">
    <ReactiveSequence>
      <SetMotion motion="OCCUPY"/>
      <SelectGoalForOccupy goal_id="{goal_id}" goal_point="{goal_point}"/>
      <PublishGoalPoint topic="clicked_point" point="{goal_point}"/>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="DoSupply">
    <ReactiveSequence>
      <SetMotion motion="SUPPLY"/>
      <SelectGoalForSupply goal_id="{goal_id}" goal_point="{goal_point}"/>
      <PublishGoalPoint topic="clicked_point" point="{goal_point}"/>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="DoRespawn">
    <ReactiveSequence>
      <SetMotion motion="RESPAWN"/>
      <WaitRespawnDone/>              <!-- 复活计时/血量恢复判断 -->
      <ClearArriveLatch/>             <!-- 清掉到达/复活相关 latch -->
      <Success/>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```


#### 黑板字段表
1) 裁判系统输入（Referee → Blackboard）

|字段名|类型|来源|说明|
|-----|----|----|----|
|ref.game_progress	|int / enum	|裁判话题/串口解析	|比赛阶段：未开始/进行中/结束（对应 game_in_progress() / game_not_start_end() 判定）|
|ref.remain_hp|	int|	裁判|	当前血量（用于 IsSentryDead/IsSentryInDanger/IsSentryRecovered）|
|ref.bullet_remain|	int|	裁判|	弹丸剩余（用于 NotBulletSufficient）|
|ref.friendly_score|	int|	裁判|	我方得分（用于优势判定）|
|ref.enemy_score|	int|	裁判|	敌方得分（用于优势判定）|
|ref.red_dead / ref.blue_dead|	int/bool|	裁判|	双方死亡状态统计（若原策略用到了）|
|ref.central_occupation|	int/enum|	裁判|	中央占领状态（若原策略用到 get_central_occupation()）|

2) 导航输入（Navigation → Blackboard）

字段名|	类型|	来源|	说明|
|-----|----|----|----|
nav.arrived	|bool|	/dstar_status 或导航回报	|“到达目标”语义（原策略里“导航部分只引用判断到达”）|
nav.pose	|geometry_msgs/PoseStamped / 自定义	|定位/TF	|如需要基于位置做更细策略（可选）|
nav.goal_active	|bool	|内部维护	|是否当前有有效目标（避免重复发布/抖动）|

3) 策略内部派生状态

字段名|	类型|	更新方式|	说明|
|-----|----|----|----|
action|string/enum|	SetAction|	当前大状态：INIT/PUSH/SIDEATTACK/OCCUPY/SUPPLY/RESPAWN
motion|string/enum|	SetMotion|	运动模式（给调试/下游用）
is_aggressive|bool|	参数/策略内部|	对应原代码 is_aggressive
is_dead|bool|	UpdateDerivedFlags|	由 ref.remain_hp 等派生
is_in_danger|bool|	UpdateDerivedFlags|	由 hp 阈值派生
bullet_sufficient|	bool|	UpdateDerivedFlags|	由 ref.bullet_remain 派生
central_occupiable|	bool|	UpdateCentralOccupiable/派生|	对应原策略“连续多秒未受攻击且中央未被占领”
central_occupiable_cnt|	int|	UpdateTimersBB|	原代码同名计数器（用于去抖/持续性判定）
respawn_waiting_cnt|	int|	UpdateTimersBB|	原代码同名（复活等待计数）
arrive_latch|	bool|	内部|	“到达事件”锁存（避免一次到达触发多次切点）
last_goal_id|	int|	内部|	上一次发布的目标 ID（避免重复发布）

4) 策略输出

字段名|	类型|	去向|	说明|
|-----|----|----|----|
goal_id|	int|	可选话题 /strategy/goal_id|	如果你希望“策略输出 ID，由桥接节点映射坐标表”|
goal_point|	geometry_msgs/PointStamped / Point|	clicked_point|	直接给 D* 的目标点（最小改动路径）|
publish_goal|	bool|	内部|	是否需要发布（例如只在目标变化时 publish）|

### V1
在目前的结构中，`./sim_nav/src/boy_sim/src/ser2msg_tf_decision_givepoint.cpp`负责和下位机通讯，通过接收下位机的决策模块发送的目标点index,在代码中根据索引确定写死的坐标点，同时持续和寻路模块`dstarlite.cpp`交互，获取当前机器人位置和目标点位置，调用A*寻路算法计算路径，和运动参数(x,y,omega)，再通过串口发送运动参数和到达情况给下位机。

你的任务是将决策模块整合到上位机中，具体要求如下：
- `./Strategy_Task.c`是过去下位机的决策代码
 - 这个代码中包含了和裁判系统交互的模块, 导航交互的模块，视觉交互的模块 (其中视觉部分没有引用，导航部分只引用判断到达)
    ```
    extern referee_info_t referee_info;
    extern vision_rx_t vision_rx;
    extern navigation_rx_t navigation_rx;
    ```
    通过extern关键字指出，其中，和裁判模块的通信可以参考`Referee_Task.c`
- 使用`behavior_tree`的方式重构`Strategy_Task.c`中的决策逻辑，你需要使用`BehaviorTree.CPP`库来实现
- 可能要在`./sim_nav/src/bot_sim/src/`目录下新建一个`strategy_node.cpp`文件，作为行为树的节点实现

在开始修改前 告诉我你的计划


### V0
#### Ans
下面基于当前工作区结构（按“模块/目录层级”）给出**目录树**与**功能说明**。其中 `build/`、`devel/` 多为 **catkin/cmake 自动生成产物**（例如大量 `*_generate_messages_*` 目标，见 [Navigation-filter-test/ws_cloud/build/Makefile](Navigation-filter-test/ws_cloud/build/Makefile)），通常不建议手改。

##### 目录树（概览）

```text
HKU_Astar_Nav_25/
├── [3DNavUL_Test.launch](http://_vscodecontentref_/0)
├── [3DSlamFinal_lio.launch](http://_vscodecontentref_/1)
├── [tf_test.rviz](http://_vscodecontentref_/2)
├── [prompts.md](http://_vscodecontentref_/3)
├── .vscode/
│   └── settings.json
├── Navigation-filter-test/
│   └── ws_cloud/
│       ├── .catkin_workspace
│       ├── .vscode/
│       │   ├── c_cpp_properties.json
│       │   └── settings.json
│       ├── src/
│       │   └── (catkin 工作空间源码：含 livox_cloudpoint_processor 等包)
│       ├── build/
│       │   └── (CMake/catkin 构建产物：Makefile、CMakeCache、各包 CMakeFiles 等)
│       └── devel/
│           └── (开发空间产物：setup 脚本、生成的消息/头文件、环境变量等)
└── sim_nav/
    └── src/
        ├── .gitattributes
        ├── [CMakeLists.txt](http://_vscodecontentref_/4)
        ├── bot_sim/
        ├── fast_gicp/
        ├── hdl_global_localization/
        ├── hdl_graph_slam/
        ├── hdl_localization/
        ├── ndt_omp/
        └── Point-LIO/
```
#### Prompt
- 你是一名精通机器人导航开发的工程师，在这个任务中，你需要在ubuntu20.04下基于ros-noetic版本进行开发，在大部分时间下你应当适用C++构建包，但在特殊说明下，你需要使用python构建脚本。
- 除非提示词特定说明进行代码的修改，不要改动工作区
- 使用bash命令行，确保你的操作是可逆的，并给出复原方案
- 对于整个文件/文件夹/包 的删除是禁止的，如果需要，告知用户且说明理由
- 你不需要通过bash来编译或执行，如果有必要，告知用户

现在首先浏览一遍工作区，分析文件结构，总结每个模块的功能和作用，列出文件目录树，并给出简要说明
- `3DNavUL_Test.launch`: 这个启动文件包含了重要的包（也就是 实际起作用的）