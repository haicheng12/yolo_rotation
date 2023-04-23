#include "opencv_test/yolo.h"
#define USE_CUDA true // use opencv-cuda
#include <cmath>
#define M_PI 3.14159265358979323846

#include <deque> //(1)

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher angle_pub_;
ros::Subscriber image_sub_;

bool Yolo::readModel(cv::dnn::Net &net, std::string &netPath, bool isCuda = false)
{
	try
	{
		net = cv::dnn::readNet(netPath);
	}
	catch (const std::exception &)
	{
		return false;
	}
	// cuda
	if (isCuda)
	{
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
	}
	// cpu
	else
	{
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	}
	return true;
}

void imageCb(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr; // 接受到ros图像的对象
	try
	{
                //TODO 编译问题没解决，建议直接程序打开摄像头
		//cv_ptr = cv_bridge::toCvCopy(msg, sensors_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// // Draw an example circle on the video stream
	// // cv_ptr->image就是传递一个Mat类当opencv使用就可以了
	// if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	// {
	// 	cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
	// }
}

void Yolo::videoDetect(std::string &videoName)
{

	if (readModel(net, model_path, USE_CUDA))
	{
		std::cout << "read net ok!" << std::endl;
	}
	else
	{
		std::cout << "read onnx model failed!";
	}

	cv::VideoCapture cap;
	cap.open(videoName);
	if (!cap.isOpened())
	{
		std::cout << "video open failure" << std::endl;
	}

	ros::NodeHandle nh;

	// Subscrive to input video feed and publish output video feed
	image_sub_ = nh.subscribe("/camera/image_raw", 1, imageCb);
	angle_pub_ = nh.advertise<std_msgs::Float64>("/angle", 10); // 发布速度

	ros::Rate loop_rate(10);
	while (ros::ok())
	// while (true)
	{
		cv::Mat SrcImg;
		cap >> SrcImg;
		cv::Mat blob;						  // 创建高维的mat，也就是四通道(batchsize,c,h,w)
		int col = SrcImg.cols;				  // 宽度
		int row = SrcImg.rows;				  // 高度
		int maxLen = MAX(col, row);			  // 取行宽最大值
		cv::Mat netInputImg = SrcImg.clone(); // 克隆一张原图，不改变原图
		if (maxLen > 1.2 * col || maxLen > 1.2 * row)
		{
			cv::Mat resizeImg = cv::Mat::zeros(maxLen, maxLen, CV_8UC3); // 重新Mat一张图片
			// 输入四个参数时，依次是x，y，w，h 从（0，0）点开始
			SrcImg.copyTo(resizeImg(cv::Rect(0, 0, col, row))); // 把srcimg的内容拷贝到resizeimg里面
			netInputImg = resizeImg;
		}
		std::vector<cv::Ptr<cv::dnn::Layer>> layer; // 创建一个容器存放layer
		std::vector<std::string> layer_names;		// 存放对应的layer的名字
		layer_names = net.getLayerNames();

		// 输入，输出，归一化，尺寸大小，三通道需要减去的值为0则不变，把图片转换成BRG通道，裁剪
		cv::dnn::blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(0, 0, 0), true, false);
		// 如果在其他设置没有问题的情况下但是结果偏差很大，可以尝试下用下面两句语句
		//  blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(104, 117, 123), true, false);
		//  blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(114, 114,114), true, false);
		net.setInput(blob);
		std::vector<cv::Mat> netOutputImg;

		net.forward(netOutputImg, net.getUnconnectedOutLayersNames());
		// net.getUnconnectedOutLayersNames()获取网络输出层信息（所有输出层的名字）每一个onnx的这个值是固定的貌似是类别+2
		// net.forward()得到各个输出层的、各个检测框等信息，是二维结构。
		std::vector<int> classIds;		// 结果id数组
		std::vector<float> confidences; // 结果每个id对应置信度数组
		std::vector<cv::Rect> boxes;	// 每个id矩形框
		float ratio_h = (float)netInputImg.rows / netHeight;
		float ratio_w = (float)netInputImg.cols / netWidth;
		int net_width = className.size() + 5; // 输出的网络宽度是类别数+5，那么这里就是6*6
		// std::vector<int> coordinates;
		for (int stride = 0; stride < strideSize; stride++)
		{													   // stride  步长这个循环的意思就是以步长为0，1，2分别去扫描这个图片
			float *pdata = (float *)netOutputImg[stride].data; // 是一个指针，指向浮点类型

			int grid_x = (int)(netWidth / netStride[stride]);
			int grid_y = (int)(netHeight / netStride[stride]);
			for (int anchor = 0; anchor < 3; anchor++)
			{ // anchors、、等同于：预定义边框 就是一组预设的边框，
				// 在训练时，以真实的边框位置相对于预设边框的偏移来构建训练样本。
				// 这就相当于，预设边框先大致在可能的位置“框“出来目标，然后再在这些预设边框的基础上进行调整。
				const float anchor_w = netAnchors[stride][anchor * 2];
				const float anchor_h = netAnchors[stride][anchor * 2 + 1];
				for (int i = 0; i < grid_y; i++)
				{
					for (int j = 0; j < grid_x; j++)
					{
						float box_score = sigmoid_x(pdata[4]);
						// 获取每一行的box框中含有某个物体的概率
						if (box_score >= boxThreshold)
						{
							cv::Mat scores(1, className.size(), CV_32FC1, pdata + 5);
							cv::Point classIdPoint;
							double max_class_socre;

							minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
							max_class_socre = sigmoid_x(max_class_socre);
							//	float classThreshold = 0.25;boxThreshold = 0.25;nmsThreshold = 0.45;
							if (max_class_socre >= classThreshold)
							{
								float x = (sigmoid_x(pdata[0]) * 2.f - 0.5f + j) * netStride[stride]; // x
								float y = (sigmoid_x(pdata[1]) * 2.f - 0.5f + i) * netStride[stride]; // y
								float w = powf(sigmoid_x(pdata[2]) * 2.f, 2.f) * anchor_w;			  // w
								float h = powf(sigmoid_x(pdata[3]) * 2.f, 2.f) * anchor_h;			  // h
								int left = (int)(x - 0.5 * w) * ratio_w + 0.5;
								int top = (int)(y - 0.5 * h) * ratio_h + 0.5;

								classIds.push_back(classIdPoint.x);
								confidences.push_back(max_class_socre * box_score);
								boxes.push_back(cv::Rect(left, top, int(w * ratio_w), int(h * ratio_h)));
							}
						}
						pdata += net_width; // 下一行
					}
				}
			}
		}
		std::vector<int> nms_result;
		cv::dnn::NMSBoxes(boxes, confidences, nmsScoreThreshold, nmsThreshold, nms_result);
		std::vector<Output> output;

		std::deque<double> degree_history; //(2)

		for (int i = 0; i < nms_result.size(); i++)
		{
			int idx = nms_result[i];
			Output result;
			result.id = classIds[idx];
			result.confidence = confidences[idx];
			result.box = boxes[idx];
			output.push_back(result);

			auto degree = atan((double)result.box.height / result.box.width);

			double degree_in_degrees = 90 - degree * 180 / M_PI; //(3)
			degree_history.push_back(degree_in_degrees);		 //(4)

			std::cout << "The Angle is: " << degree_in_degrees << " degree\n";

			std_msgs::Float64 angle_msg;
			angle_msg.data = degree_in_degrees;
			angle_pub_.publish(angle_msg);

			if (degree_history.size() > 6)
			{
				degree_history.pop_front();
			} //(5)

			bool four_consecutive = true;
			for (double d : degree_history)
			{
				if (d >= 10)
				{
					four_consecutive = false;
					break;
				}
			}

			if (four_consecutive)
			{
				std::cout << "OK, it's time to stop" << std::endl;
				break;
			} //(5)
		}
		std::vector<cv::Scalar> color;
		srand(time(0));
		for (int i = 0; i < 80; i++)
		{
			int b = rand() % 256;
			int g = rand() % 256;
			int r = rand() % 256;
			color.push_back(cv::Scalar(b, g, r));
		}
		drawPred(SrcImg, output, color);
		output.clear();

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void Yolo::drawPred(cv::Mat &img, std::vector<Output> result, std::vector<cv::Scalar> color)
{
	for (int i = 0; i < result.size(); i++)
	{
		int left, top;
		left = result[i].box.x;
		top = result[i].box.y;
		// locality.push_back({ left, top });
		int color_num = i;
		rectangle(img, result[i].box, color[result[i].id], 2, 8);
		std::string label = className[result[i].id] + ":" + std::to_string(result[i].confidence);

		int baseLine;
		cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
		top = std::max(top, labelSize.height);
		putText(img, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 1, color[result[i].id], 2);
	}

	cv::imshow("video", img);
	cv::waitKey(1);
}
