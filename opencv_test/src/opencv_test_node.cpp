// #include <ros/ros.h>
// #include <stdio.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>

// static const char WINDOW[] = "Image window";
//  static void help()
//  {
//    printf("\nThis program demonstrates converting OpenCV Image to ROS Image messages  \n");
//  }

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  // help();
  ros::init(argc, argv, "image_converter");

  // Reading an image from the file
  // cv::Mat cv_image = cv::imread("/home/ubuntu/1.jpg");
  // if (cv_image.empty())
  // {
  //   ROS_ERROR("Read the picture failed!");
  //   return -1;
  // }

  // Convert OpenCV image to ROS message
  ros::NodeHandle node;
  // image_transport::ImageTransport transport(node);
  // image_transport::Publisher image_pub;
  // image_pub = transport.advertise("OutImage", 1);
  // ros::Time time = ros::Time::now();

  // cv_bridge::CvImage cvi;
  // cvi.header.stamp = time;
  // cvi.header.frame_id = "image";
  // cvi.encoding = "bgr8";
  // cvi.image = cv_image;

  // sensor_msgs::Image im;
  // cvi.toImageMsg(im);
  // image_pub.publish(im);
  // ROS_INFO("Converted Successfully!");

  // // Show the image
  // cv::namedWindow(WINDOW);
  // cv::imshow(WINDOW, cv_image);
  // cv::waitKey(0);

  // system("color F0"); // 更改输出界面颜色
  //  VideoCapture video("/home/ubuntu/yolo/video/3.mp4");
  VideoCapture video;
  video.open(0);
  if (video.isOpened())
  {
    cout << "视频中图像的宽度=" << video.get(CAP_PROP_FRAME_WIDTH) << endl;
    cout << "视频中图像的高度=" << video.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "视频帧率=" << video.get(CAP_PROP_FPS) << endl;
    cout << "视频的总帧数=" << video.get(CAP_PROP_FRAME_COUNT);
  }
  else
  {
    cout << "请确认视频文件名称是否正确" << endl;
    return -1;
  }

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    Mat frame;
    video >> frame;
    if (frame.empty())
    {
      break;
    }
    imshow("video", frame);
    waitKey(1000 / video.get(CAP_PROP_FPS));

    ros::spinOnce();
    loop_rate.sleep();
  }
  waitKey();
  return 0;
}
