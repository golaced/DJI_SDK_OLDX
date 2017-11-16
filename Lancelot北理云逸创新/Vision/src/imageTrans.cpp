/*
 * imageTrans.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: odroid
 */

#include "imageTrans.h"
#include "trace.h"

using namespace std;
using namespace cv;

void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle)
{
	CV_Assert(!inputIm.empty());
	Mat inputImg;
	inputIm.copyTo(inputImg);
	float radian = (float)(angle / 180.0 * CV_PI);
	int uniSize = (int)(max(inputImg.cols, inputImg.rows) * 1.414);
	int dx = (int)(uniSize - inputImg.cols) / 2;
	int dy = (int)(uniSize - inputImg.rows) / 2;
	copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
	Point2f center((float)(tempImg.cols / 2), (float)(tempImg.rows / 2));
	Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
	warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
	float sinVal = fabs(sin(radian));
	float cosVal = fabs(cos(radian));
	Size targetSize((int)(inputImg.cols * cosVal + inputImg.rows * sinVal),
					(int)(inputImg.cols * sinVal + inputImg.rows * cosVal));
	int x = (tempImg.cols - targetSize.width) / 2;
	int y = (tempImg.rows - targetSize.height) / 2;
	Rect rect(x, y, targetSize.width, targetSize.height);
	tempImg = Mat(tempImg, rect);
}

void drawImage(cv::Mat &image)
{
	char temp_text[50];
	int char_to_recognition = 0;

	static Point pt_src_center(image.cols / 2, image.rows / 2);
	sprintf(temp_text, "target=%d", target_num);
	putText(image, temp_text, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.6,
			CV_RGB(255, 0, 0), 2);
	sprintf(temp_text, "Pit=%.2f", Pitch);
	putText(image, temp_text, Point(10, 80), FONT_HERSHEY_SIMPLEX, 0.6,
			CV_RGB(255, 0, 0), 2);
	sprintf(temp_text, "Yaw=%.2f", Yaw);
	putText(image, temp_text, Point(10, 100), FONT_HERSHEY_SIMPLEX, 0.6,
			CV_RGB(255, 0, 0), 2);
	sprintf(temp_text, "Roll=%.2f", Roll);
	putText(image, temp_text, Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.6,
			CV_RGB(255, 0, 0), 2);
	sprintf(temp_text, "  %d", state_num);
	putText(image, state_str[state_num] + temp_text, Point(160, 20),
			CV_FONT_HERSHEY_TRIPLEX, 0.6, CV_RGB(0, 255, 0), 1, 8);

	putText(image, number_position_send.number_, Point(10, 140),
			CV_FONT_HERSHEY_TRIPLEX, 0.6, CV_RGB(255, 0, 0), 1, 8);

	sprintf(temp_text, "pt=%d %d", number_position_send.position_.x,
			number_position_send.position_.y);
	putText(image, temp_text, Point(10, 160), FONT_HERSHEY_SIMPLEX, 0.6,
			CV_RGB(255, 0, 0), 2);

	cv::line(image, pt_src_center - Point(10, 0), pt_src_center + Point(10, 0),
			 CV_RGB(0, 255, 0), 2);
	cv::line(image, pt_src_center - Point(0, 10), pt_src_center + Point(0, 10),
			 CV_RGB(0, 255, 0), 2);
}

void startWriteVideo(cv::VideoWriter &video_writer)
{
	string user_path = expand_user("~");
	string video_num_path(user_path + "/Lancelot/video/video_num.txt");

	int video_num = 0;
	std::ifstream video_num_read;
	video_num_read.open(video_num_path.c_str());
	video_num_read >> video_num;
	video_num_read.close();

	cout << video_num << endl;

	std::ofstream video_num_write;
	video_num_write.open(video_num_path.c_str());
	video_num_write << (video_num + 1);
	video_num_write.close();

	if (video_writer.isOpened())
	{
		video_writer.release();
	}

	std::stringstream ss;
	string video_name;

	ss << video_num;
	ss >> video_name;
	video_name += ".avi";

	video_writer.open(user_path + "/Lancelot/video/" + video_name,
					  CV_FOURCC('D', 'I', 'V', 'X'), 15, Size(320, 240));
}
