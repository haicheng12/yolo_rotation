#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include <math.h>
#include "std_msgs/Float64.h"

ros::Publisher cmd_vel_pub_;

// 状态机
enum RotateState // 小车状态机
{
  ROTATION_ALIGNMENT = 1, // 旋转对准
  ROTATION_FINISH         // 旋转完成
};
RotateState rotation_state_;

geometry_msgs::PoseStamped current_pose_msg;
geometry_msgs::TwistStamped current_vel_msg;

bool is_sub_odom = false;
bool is_sub_angle = false;

double Position_KP = 0.5; // 位置式PID控制器参数设定
double Position_KI = 0.0;
double Position_KD = 0.5;

double camera_angle_; // 相机传过来的角度

inline double angleLimit(double yaw) // 角度限制
{
  double theta = yaw;
  if (theta > M_PI)
  {
    theta = theta - 2 * M_PI;
  }
  else if (theta < -M_PI)
  {
    theta = theta + 2 * M_PI;
  }
  return theta;
}

// void odomCallback(const nav_msgs::Odometry &msg)
// {
//   current_pose_msg.pose = msg.pose.pose;
//   current_vel_msg.twist = msg.twist.twist;

//   is_sub_odom = true;
// }

void angleCallback(const std_msgs::Float64 &msg)
{
  camera_angle_ = msg.data;
  camera_angle_ = angleLimit(camera_angle_ * M_PI / 180);

  current_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(camera_angle_); // 欧拉角转四元数

  is_sub_angle = true;
}

double Xianfu(double value, double Amplitude) // 限制最大速度幅度
{
  double temp;
  if (value > Amplitude)
    temp = Amplitude;
  else if (value < -Amplitude)
    temp = -Amplitude;
  else
    temp = value;

  return temp;
}

// 函数功能：位置式PID控制器
// 入口参数：当前位置，目标位置
// 返回  值：控制的速度
// 根据位置式离散PID公式
// Speed=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
// e(k)代表本次偏差
// e(k-1)代表上一次的偏差
// ∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
// Speed代表输出
double Position_PID(double Position, double target)
{
  static double Bias, Speed, Integral_bias, Last_Bias;
  Bias = target - Position; // 计算偏差
  Integral_bias += Bias;    // 求出偏差的积分
  Integral_bias = Xianfu(Integral_bias, 0.5);
  Speed = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // 位置式PID控制器
  Last_Bias = Bias;                                                                            // 保存上一次偏差

  if (Speed > 0.5)
    Speed = 0.5;
  if (Speed < -0.5)
    Speed = -0.5;

  return Speed; // 增量输出
}

RotateState calculateAngle(double current_angle, double goal_angle)
{
  double angle_dis = fabs(angleLimit(current_angle - goal_angle));
  ROS_INFO("angle_dis [%f]", angle_dis);

  if (angle_dis <= 5 * M_PI / 180.0) // 小于一定范围表示旋转完成
  {
    return RotateState::ROTATION_FINISH; // 小车状态：旋转完成
  }
  else
  {
    return RotateState::ROTATION_ALIGNMENT; // 小车状态：旋转对准
  }
  return RotateState::ROTATION_FINISH;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotation_alignment");
  ros::NodeHandle nh;

  // ros::Subscriber current_pose_sub = nh.subscribe("/odom", 10, odomCallback); // 接收车辆位置
  ros::Subscriber angle_sub = nh.subscribe("/angle", 10, angleCallback); // 接收角度
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);     // 发布速度

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // if (is_sub_odom)
    if (is_sub_angle)
    {
      rotation_state_ = calculateAngle(tf::getYaw(current_pose_msg.pose.orientation), 0.0); // 传入目标角度

      double angle = tf::getYaw(current_pose_msg.pose.orientation) - 0.0;
      ROS_INFO("angle [%f]", angle);

      switch (rotation_state_)
      {
      case ROTATION_ALIGNMENT:
      {
        ROS_INFO("ROTATION_ALIGNMENT");
        double position_motor = Position_PID(-angle, 0.0);
        double limit_a = Xianfu(position_motor, 0.4);

        geometry_msgs::Twist vel_msg; // 发布速度
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = -limit_a;
        cmd_vel_pub_.publish(vel_msg);
        ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);
        break;
      }

      case ROTATION_FINISH:
      {
        ROS_INFO("ROTATION_FINISH");
        geometry_msgs::Twist vel_msg; // 发布速度
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        cmd_vel_pub_.publish(vel_msg);
        ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);
        break;
      }

      default:
        break;
      }
      // is_sub_odom = false;
      is_sub_angle = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
