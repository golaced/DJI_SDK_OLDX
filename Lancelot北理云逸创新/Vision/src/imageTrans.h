/*
 * imageTrans.h
 *
 *  Created on: Sep 14, 2016
 *      Author: odroid
 */

#ifndef IMAGETRANS_H_
#define IMAGETRANS_H_

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle);

void drawImage(cv::Mat &image);

void startWriteVideo(cv::VideoWriter &video_writer);

#endif /* IMAGETRANS_H_ */
