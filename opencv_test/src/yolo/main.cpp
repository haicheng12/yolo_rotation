#include "opencv_test/yolo.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
// #include <Windows.h>
#include <ros/ros.h>

using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "yolo");

	Yolo test;

	test.model_path = "/home/ubuntu/yolo/models/best.onnx";
	std::string videoname = "/home/ubuntu/yolo/video/3.mp4";

	// test.model_path = "./models/best.onnx";
	// std::string videoname = "./video/1.mp4";
	test.videoDetect(videoname);

	// system("pause");
	return 0;
}
