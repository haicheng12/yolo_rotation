#pragma once
#include<iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

struct Output 
{
	int id;             //������id
	float confidence;   //������Ŷ�
	cv::Rect box;       //���ο�
};

class Yolo 
{
protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
        //image_transport::Publisher angle_pub_;
        ros::Publisher angle_pub_;

        void imageCb(const sensor_msgs::ImageConstPtr &msg);

        cv::Mat image_;
public:
	Yolo();
	~Yolo();

	bool readModel(cv::dnn::Net& net, std::string& netPath, bool isCuda);
	void drawPred(cv::Mat& img, std::vector<Output> result, std::vector<cv::Scalar> color);
	void videoDetect(std::string& videoName);
       
	std::string model_path;
	cv::dnn::Net net;

private:

	float sigmoid_x(float x)
	{
		return static_cast<float>(1.f / (1.f + exp(-x)));
	}

	const float netAnchors[3][6] = { { 10, 13, 16, 30, 33, 23 }, { 30, 61, 62, 45, 59, 119 }, { 116, 90, 156, 198, 373, 326 } };//yolov5-P5 anchors
	//const float (int) ��˼���ǳ��������޸�
	const int netWidth = 640;   //ONNXͼƬ�������
	const int netHeight = 640;  //ONNXͼƬ����߶�
	const int strideSize = 3;   //stride size


	const float netStride[4] = { 8, 16.0, 32, 64 };

	float boxThreshold = 0.25;
	float classThreshold = 0.25;
	float nmsThreshold = 0.45;
	float nmsScoreThreshold = boxThreshold * classThreshold;

	//std::vector<std::string> className = { "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
	//	"fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
	//	"elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
	//	"skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
	//	"tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
	//	"sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
	//	"potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
	//	"microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
	//	"hair drier", "toothbrush" };
	std::vector<std::string> className = { "pipe" };

};
