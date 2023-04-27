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
#include "std_msgs/Bool.h"

ros::Publisher cmd_vel_pub_;

// 状态机
enum RotateState // 小车状态机
{
  ROTATION_ALIGNMENT = 1, // 旋转对准
  ROTATION_FINISH         // 旋转完成
};
RotateState rotation_state_;

bool is_sub_count = false;

double camera_count_; // 相机传过来的标志位

void countCallback(const std_msgs::Bool &msg)
{
  camera_count_ = msg.data;

  is_sub_count = true;
}

RotateState calculateCount(bool count_flag)
{
  if (count_flag) // 表示旋转完成
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

  ros::Subscriber count_sub = nh.subscribe("/stop_flag", 1, countCallback);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // 发布速度

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (is_sub_count)
    {
      rotation_state_ = calculateCount(camera_count_); // 传入标志位

      switch (rotation_state_)
      {
      case ROTATION_ALIGNMENT:
      {
        geometry_msgs::Twist vel_msg; // 发布速度
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.3;
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
      is_sub_count = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}