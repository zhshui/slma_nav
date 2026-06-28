/**********************************************************************
 * 改动说明：
 * 1. 引入了 ros/ros.h 和 geometry_msgs/Twist.h
 * 2. 删除了固定的 TEST_MODE 循环逻辑，改为由回调函数驱动
 * 3. 增加了对 cmd_vel 话题的订阅
 * 4. EMA 低通滤波 + 角速度缩放，支持运行时动态调参
 ***********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

// ============================================================
// 一阶指数移动平均（EMA）低通滤波器
//   filtered = alpha * raw + (1 - alpha) * filtered_prev
//   首次调用直接使用原始值（无历史时不滤波）
// ============================================================
class EmaFilter
{
public:
  EmaFilter(double alpha = 0.3) : alpha_(alpha), initialized_(false), filtered_(0.0) {}

  double filter(double raw)
  {
    if (!initialized_)
    {
      filtered_ = raw;
      initialized_ = true;
    }
    else
    {
      filtered_ = alpha_ * raw + (1.0 - alpha_) * filtered_;
    }
    return filtered_;
  }

  void reset()
  {
    initialized_ = false;
    filtered_ = 0.0;
  }

  void setAlpha(double alpha) { alpha_ = alpha; }

private:
  double alpha_;
  bool initialized_;
  double filtered_;
};

class Custom
{
public:
  explicit Custom(ros::NodeHandle& nh) : nh_(nh)
  {
    // 初始化 Unitree SDK 客户端
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // 订阅机器人内部 DDS 状态（可选，用于获取反馈）
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

    // 从 ROS param 读取滤波器初始参数
    nh_.param("smoothing_alpha_linear", alpha_linear_, 0.3);
    nh_.param("smoothing_alpha_angular", alpha_angular_, 0.3);
    nh_.param("angular_scale", angular_scale_, 1.5);
    filter_linear_x_.setAlpha(alpha_linear_);
    filter_linear_y_.setAlpha(alpha_linear_);
    filter_angular_.setAlpha(alpha_angular_);
  }

  // ROS cmd_vel 回调函数（EMA 滤波 + 角速度缩放）
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    double vx_raw = msg->linear.x;    // 前进后退速度
    double vy_raw = msg->linear.y;    // 左右平移速度
    double vyaw_raw = msg->angular.z; // 旋转角速度

    // 每轮回调重读 ROS param，支持运行时动态调参
    nh_.param("smoothing_alpha_linear", alpha_linear_, 0.3);
    nh_.param("smoothing_alpha_angular", alpha_angular_, 0.3);
    nh_.param("angular_scale", angular_scale_, 1.5);
    filter_linear_x_.setAlpha(alpha_linear_);
    filter_linear_y_.setAlpha(alpha_linear_);
    filter_angular_.setAlpha(alpha_angular_);

    // EMA 低通滤波（vx/vy 各自独立滤波器，互不污染）
    double vx = filter_linear_x_.filter(vx_raw);
    double vy = filter_linear_y_.filter(vy_raw);
    double vyaw = filter_angular_.filter(vyaw_raw) * angular_scale_;

    // 调用 SDK 的 Move 接口
    sport_client.Move(vx, vy, vyaw);

    ROS_INFO("CmdVel raw:[%.3f, %.3f, %.3f] -> filtered:[%.3f, %.3f, %.3f] scale:%.2f",
             vx_raw, vy_raw, vyaw_raw, vx, vy, vyaw, angular_scale_);
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
      // 停止时自动清零全部滤波器
      filter_linear_x_.reset();
      filter_linear_y_.reset();
      filter_angular_.reset();
      ROS_INFO(" -> StopMove (filters reset)");
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

private:
  ros::NodeHandle nh_;
  EmaFilter filter_linear_x_;
  EmaFilter filter_linear_y_;
  EmaFilter filter_angular_;
  double alpha_linear_;
  double alpha_angular_;
  double angular_scale_;
};

int main(int argc, char **argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "go2_cmd_vel_bridge");
  ros::NodeHandle nh("~");  // 使用私有命名空间读取 param

  // 获取网卡参数 (从 launch 文件或命令行传入)
  std::string net_interface = "eth0";
  if (argc > 1) {
    net_interface = argv[1];
  }
  nh.param<std::string>("nic", net_interface, net_interface);

  // 初始化 Unitree 通道
  unitree::robot::ChannelFactory::Instance()->Init(0, net_interface);

  Custom custom(nh);

  // 创建 ROS 订阅者，监听 /cmd_vel 话题（速度控制）
  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, &Custom::CmdVelCallback, &custom);

  // 创建 ROS 订阅者，监听 /go2/sport_cmd 话题（姿态指令）
  ros::Subscriber sport_sub = nh.subscribe("/go2/sport_cmd", 10, &Custom::SportCmdCallback, &custom);

  ROS_INFO("Go2 CMD_VEL Bridge Started (EMA filter enabled). Listening to /cmd_vel and /go2/sport_cmd...");

  // 让机器站起来，准备接受运动指令
  sleep(1);
  custom.sport_client.StandUp();
  ROS_INFO("Robot standing up...");

  // 循环等待 ROS 回调
  ros::spin();

  return 0;
}