/*
 * main.cpp
 *
 *  Created on: 2017年10月25日
 *      Author: yfji
 */

#include "DigitDetector.h"
#include "TwoLayerNNFaster.h"

vector<std::tuple<int, double, cv::Rect>> findPrintDigitAreas(TwoLayerNNFaster &nn, DigitDetector &detector, Mat &frame);

vector<std::tuple<int, double, cv::Rect>> findLedDigitAreas(TwoLayerNNFaster &nn, DigitDetector &detector, Mat &frame);