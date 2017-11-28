/*
 * main.cpp
 *
 *  Created on: 2017年10月25日
 *      Author: yfji
 */

#include "DigitDetector.h"
#include "TwoLayerNNFaster.h"

cv::Mat biMap, not_biMap;

vector<std::tuple<int, double, cv::Rect>> findPrintDigitAreas(TwoLayerNNFaster &nn, DigitDetector &detector, Mat &frame)
{
	detector.binarize(frame, biMap);
	cv::bitwise_not(biMap, not_biMap);
	vector<Rect> digits = detector.getCandidateBndBoxes(not_biMap);
	vector<std::tuple<int, double, cv::Rect>> res;
	for (size_t i = 0; i < digits.size(); ++i)
	{
		Rect &r = digits[i];
		Mat roi = biMap(r);
		int pred;
		double prob;
		nn.predict(roi, pred, prob);
		if (pred >= 0 and pred < 10 and prob > 0.9)
		{
			res.push_back(std::make_tuple(pred, prob, Rect(r)));
		}
	}
	return res;
}

vector<std::tuple<int, double, cv::Rect>> findLedDigitAreas(TwoLayerNNFaster &nn, DigitDetector &detector, Mat &frame)
{
	detector.binarize(frame, biMap);
	cv::bitwise_not(biMap, not_biMap);
	vector<Rect> digits = detector.getCandidateBndBoxes(biMap);
	vector<std::tuple<int, double, cv::Rect>> res;
	for (size_t i = 0; i < digits.size(); ++i)
	{
		Rect &r = digits[i];
		Mat roi = not_biMap(r);
		int pred;
		double prob;
		nn.predict(roi, pred, prob);
		if (pred >= 0 and pred < 10 and prob > 0.9)
		{
			res.push_back(std::make_tuple(pred, prob, Rect(r)));
		}
	}
	return res;
}