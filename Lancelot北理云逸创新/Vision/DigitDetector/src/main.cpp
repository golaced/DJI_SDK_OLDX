/*
 * main.cpp
 *
 *  Created on: 2017年10月25日
 *      Author: yfji
 */

#include "DigitDetector.h"
#include "TwoLayerNNFaster.h"

cv::Mat biMap1, not_biMap1;

vector<std::tuple<int, double, cv::Rect>> findPrintDigitAreas1(TwoLayerNNFaster &nn, DigitDetector &detector, Mat &frame)
{
	detector.binarize(frame, biMap1);
	cv::bitwise_not(biMap1, not_biMap1);
	vector<Rect> digits = detector.getCandidateBndBoxes(not_biMap1);
	vector<std::tuple<int, double, cv::Rect>> res;
	for (size_t i = 0; i < digits.size(); ++i)
	{
		Rect &r = digits[i];
		Mat roi = biMap1(r);
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

vector<std::tuple<int, double, cv::Rect>> findLedDigitAreas1(TwoLayerNNFaster &nn, DigitDetector &detector, Mat &frame)
{
	detector.binarize(frame, biMap1);
	cv::bitwise_not(biMap1, not_biMap1);
	vector<Rect> digits = detector.getCandidateBndBoxes(biMap1);
	vector<std::tuple<int, double, cv::Rect>> res;
	for (size_t i = 0; i < digits.size(); ++i)
	{
		Rect &r = digits[i];
		Mat roi = not_biMap1(r);
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

int main(int argc, char **argv)
{
	VideoCapture cap;
	cap.open("/home/auto710/Lancelot/video/实验视频/124.avi");
	//cap.open(1);
	DigitDetector detector;
	TwoLayerNNFaster nn_print(TEST);
	TwoLayerNNFaster nn_led(TEST);
	nn_print.loadParams("/home/auto710/Lancelot/DigitDetector/nn_params_hist_iter_12000.txt");
	nn_led.loadParams("/home/auto710/Lancelot/DigitDetector/params_hist_led.txt");
	int fps = cap.get(cv::CAP_PROP_FPS);
	cout << "fps: " << fps << endl;
	bool ok;
	char key = 0;
	Mat frame; //biMap, not_biMap1;
	cv::TickMeter tm;
	tm.reset();
	while (1)
	{
		ok = cap.read(frame);
		if (not ok or key == 27)
			break;
		//cv::resize(frame, frame, Size(frame.cols/2, frame.rows/2), cv::INTER_CUBIC);
		//predict(biMap(rec), pred, prob) for printed digits with black pixels
		//predict(not_biMap1(rec), pred, prob) for led digits with white pixels

		//vector<std::tuple<int,double, cv::Rect> >&& res=findLedDigitAreas(nn_led, detector, frame);
		tm.reset();
		tm.start();
		vector<std::tuple<int, double, cv::Rect>> &&res = findPrintDigitAreas1(nn_print, detector, frame);
		for (size_t i = 0; i < res.size(); ++i)
		{
			Rect &r = std::get<2>(res[i]);
			stringstream ss;
			ss << std::get<0>(res[i]) << ": " << std::get<1>(res[i]);
			cv::rectangle(frame, r, Scalar(0, 0, 255), 3);
			cv::putText(frame, ss.str(), Point(r.x, r.y - 5),
						cv::FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 3);
		}
		tm.stop();
		cout << "findPrintDigitAreas: " << tm.getTimeMilli() << " ms" << endl;
		cv::imshow("frame", frame);
		key = waitKey(10);
	}
}
