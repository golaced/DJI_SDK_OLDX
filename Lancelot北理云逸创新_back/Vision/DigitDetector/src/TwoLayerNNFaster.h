/*
 * TwoLayerNNFaster.h
 *
 *  Created on: 2017年10月23日
 *      Author: yfji
 */

#ifndef SRC_TWOLAYERNNFASTER_H_
#define SRC_TWOLAYERNNFASTER_H_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <random>
#include <cmath>
#include <cblas.h>
#include "util.h"
using namespace std;
using namespace cv;

#define TRAIN			0
#define TEST			1
#define FINETUNE	2

class TwoLayerNNFaster {
public:
	TwoLayerNNFaster(int mode=TRAIN, int minibatchIfTrain=32);
	virtual ~TwoLayerNNFaster();

public:
	void train(int iters=1e6);
	void predict(Mat& image, int& pred, double& prob);
	void saveParams(const char* filename);
	void loadParams(const char* filename);

public:
	int step;
	int display;
	double gamma;
	double lr;
	double reg;

public:
	const int nLayers[3]={150, 400, 11};
	int mode;
	int minibatch;
	double* W1;
	double* W2;
	double* gW1;
	double* gW2;
	double* gb1;
	double* gb2;
	double* b1;
	double* b2;
	//double* input;
	double* output;
	double* yLayer1;
	double* yLayer2;
	double* dLayer1;
	double* dLayer2;
	double* batchSamples;
	int* batchLabels;

	vector<string> sampleFiles;
	vector<int> labelMat;
	vector<int> ind;

	int curIndex;
	int numSamples;
	const int pool_h=15;
	const int pool_w=10;
	const int resz_h=20;
	const int resz_w=15;
	string sample_file_lst;

	void shuffle();
	double forward();
	void backward();
	void reset(double std=1e-2);
	void alloc();
	void release();
	void getFeature(Mat& image, int batchIndex);
	void getFeatureX(Mat& image, int batchIndex);
	void loadSamplePathAndLabels();
};

#endif /* SRC_TWOLAYERNNFASTER_H_ */
