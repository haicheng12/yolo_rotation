#include "opencv_test/yolo.h"
#define USE_CUDA true // use opencv-cuda
#include <cmath>
#define M_PI 3.14159265358979323846

#include <deque> //(1)

Yolo::Yolo() : it_(nh_)
{
	image_sub_ = it_.subscribe("/rgb/image_raw", 1, &Yolo::imageCb, this);
	angle_pub_ = nh_.advertise<std_msgs::Float64>("/angle", 10); // 发布速度

	cv::namedWindow(OPENCV_WINDOW);
}

Yolo::~Yolo()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void Yolo::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
	ROS_INFO_STREAM("Get Msg");
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		// TODO 未就决cv_bridge的使用
		//  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Draw an example circle on the video stream
	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);
}

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

	ros::Rate loop_rate(1);
	while (ros::ok())
	// while (true)
	{
		cv::Mat SrcImg;
		cap >> SrcImg;
		cv::Mat blob;						  // ������ά��mat��Ҳ������ͨ��(batchsize,c,h,w)
		int col = SrcImg.cols;				  // ����
		int row = SrcImg.rows;				  // �߶�
		int maxLen = MAX(col, row);			  // ȡ�п����ֵ
		cv::Mat netInputImg = SrcImg.clone(); // ��¡һ��ԭͼ�����ı�ԭͼ
		if (maxLen > 1.2 * col || maxLen > 1.2 * row)
		{
			cv::Mat resizeImg = cv::Mat::zeros(maxLen, maxLen, CV_8UC3); // ����Matһ��ͼƬ
			SrcImg.copyTo(resizeImg(cv::Rect(0, 0, col, row)));			 // ��srcimg�����ݿ�����resizeimg����//�����ĸ�����ʱ��������x��y��w��h �ӣ�0��0���㿪ʼ
			netInputImg = resizeImg;
		}
		std::vector<cv::Ptr<cv::dnn::Layer>> layer; // ����һ���������layer
		std::vector<std::string> layer_names;		// ��Ŷ�Ӧ��layer������
		layer_names = net.getLayerNames();

		cv::dnn::blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(0, 0, 0), true, false); // ���룬�������һ�����ߴ��С����ͨ����Ҫ��ȥ��ֵΪ0�򲻱䣬��ͼƬת����BRGͨ�����ü�
		// �������������û�����������µ��ǽ��ƫ��ܴ󣬿��Գ������������������
		// blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(104, 117, 123), true, false);
		// blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(114, 114,114), true, false);
		net.setInput(blob);
		std::vector<cv::Mat> netOutputImg;

		net.forward(netOutputImg, net.getUnconnectedOutLayersNames()); // net.getUnconnectedOutLayersNames()��ȡ�����������Ϣ���������������֣�ÿһ��onnx�����ֵ�ǹ̶���ò�������+2
		// net.forward()�õ����������ġ������������Ϣ���Ƕ�ά�ṹ��

		std::vector<int> classIds;		// ���id����
		std::vector<float> confidences; // ���ÿ��id��Ӧ���Ŷ�����
		std::vector<cv::Rect> boxes;	// ÿ��id���ο�
		float ratio_h = (float)netInputImg.rows / netHeight;
		float ratio_w = (float)netInputImg.cols / netWidth;
		int net_width = className.size() + 5; // �������������������+5����ô�������6*6
		// std::vector<int> coordinates;
		for (int stride = 0; stride < strideSize; stride++)
		{													   // stride  �������ѭ������˼�����Բ���Ϊ0��1��2�ֱ�ȥɨ�����ͼƬ
			float *pdata = (float *)netOutputImg[stride].data; // ��һ��ָ�룬ָ�򸡵����͡�

			int grid_x = (int)(netWidth / netStride[stride]);
			int grid_y = (int)(netHeight / netStride[stride]);
			for (int anchor = 0; anchor < 3; anchor++)
			{ // anchors������ͬ�ڣ�Ԥ����߿� ����һ��Ԥ��ı߿���ѵ��ʱ������ʵ�ı߿�λ�������Ԥ��߿��ƫ��������ѵ�������� ����൱�ڣ�Ԥ��߿��ȴ����ڿ��ܵ�λ�á��򡰳���Ŀ�꣬Ȼ��������ЩԤ��߿�Ļ����Ͻ��е�����
				const float anchor_w = netAnchors[stride][anchor * 2];
				const float anchor_h = netAnchors[stride][anchor * 2 + 1];
				for (int i = 0; i < grid_y; i++)
				{
					for (int j = 0; j < grid_x; j++)
					{
						float box_score = sigmoid_x(pdata[4]);
						; // ��ȡÿһ�е�box���к���ĳ������ĸ���
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
						pdata += net_width; // ��һ��
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
