/*
 * util.h
 *
 *      Author: JYF
 */

#ifndef SRC_UTIL_H_
#define SRC_UTIL_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>
#include <iostream>
using namespace cv;
using namespace std;

Rect findDigitBndBox(Mat& image);

int Compare (const int& a, const int& b);

void nms(vector<Rect>& rects);

#endif /* SRC_UTIL_H_ */
