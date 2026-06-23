/**********************************************************************
 * 改动说明：
 * 1. 引入了 ros/ros.h 和 geometry_msgs/Twist.h
 * 2. 删除了固定的 TEST_MODE 循环逻辑，改为由回调函数驱动
 * 3. 增加了对 cmd_vel 话题的订阅
 ***********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

class Custom
{
public:
  Custom()
  {
    // 初始化 Unitree SDK 客户端
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // 订阅机器人内部 DDS 状态（可选，用于获取反馈）
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  }

  // ROS cmd_vel 回调函数
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    double vx = msg->linear.x;   // 前进后退速度
    double vy = msg->linear.y;   // 左右平移速度
    double vyaw = msg->angular.z; // 旋转角速度

    // 调用 SDK 的 Move 接口
    sport_client.Move(vx, vy, vyaw);

    ROS_INFO("Sent move command: vx: [%f], vy: [%f], vyaw: [%f]", vx, vy, vyaw);
  }

  // ROS sport_cmd 回调: 接收 String 指令 (stand_up / sit / damp / stand_down / recovery_stand / stop_move / balance_stand)
  void SportCmdCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::string cmd = msg->data;
    ROS_INFO("Received sport_cmd: [%s]", cmd.c_str());

    if (cmd == "stand_up" || cmd == "stand") {
      sport_client.StandUp();
      ROS_INFO(" -> StandUp");
    } else if (cmd == "sit") {
      sport_client.Sit();
      ROS_INFO(" -> Sit");
    } else if (cmd == "damp") {
      sport_client.Damp();
      ROS_INFO(" -> Damp");
    } else if (cmd == "stand_down") {
      sport_client.StandDown();
      ROS_INFO(" -> StandDown");
    } else if (cmd == "recovery_stand") {
      sport_client.RecoveryStand();
      ROS_INFO(" -> RecoveryStand");
    } else if (cmd == "balance_stand") {
      sport_client.BalanceStand();
      ROS_INFO(" -> BalanceStand");
    } else if (cmd == "stop_move") {
      sport_client.StopMove();
      ROS_INFO(" -> StopMove");
    } else if (cmd == "rise_sit") {
      sport_client.RiseSit();
      ROS_INFO(" -> RiseSit");
    } else if (cmd == "hello") {
      sport_client.Hello();
      ROS_INFO(" -> Hello");
    } else {
      ROS_WARN("Unknown sport_cmd: [%s]", cmd.c_str());
    }
  }

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  }

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
};

int main(int argc, char **argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "go2_cmd_vel_bridge");
  ros::NodeHandle nh;

  // 获取网卡参数 (从 launch 文件或命令行传入)
  std::string net_interface = "eth0";
  if (argc > 1) {
    net_interface = argv[1];
  }
  nh.param<std::string>("nic", net_interface, net_interface);

  // 初始化 Unitree 通道
  unitree::robot::ChannelFactory::Instance()->Init(0, net_interface);
  
  Custom custom;

  // 创建 ROS 订阅者，监听 /cmd_vel 话题（速度控制）
  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, &Custom::CmdVelCallback, &custom);

  // 创建 ROS 订阅者，监听 /go2/sport_cmd 话题（姿态指令）
  ros::Subscriber sport_sub = nh.subscribe("/go2/sport_cmd", 10, &Custom::SportCmdCallback, &custom);

  ROS_INFO("Go2 CMD_VEL Bridge Started. Listening to /cmd_vel and /go2/sport_cmd...");

  // 让机器站起来，准备接受运动指令
  sleep(1); 
  custom.sport_client.StandUp();
  ROS_INFO("Robot standing up...");

  // 循环等待 ROS 回调
  ros::spin();

  return 0;
}