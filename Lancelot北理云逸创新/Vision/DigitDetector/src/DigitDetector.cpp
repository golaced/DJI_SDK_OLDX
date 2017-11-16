/*
 * DigitDetector.cpp
 *
 *  Created on: 2017年10月25日
 *      Author: yfji
 */

#include "DigitDetector.h"

DigitDetector::DigitDetector() {
	// TODO Auto-generated constructor stub
}

DigitDetector::~DigitDetector() {
	// TODO Auto-generated destructor stub
}

void DigitDetector::RGBToLab(unsigned char * rgbImg, float * labImg)
{
	float B = gamma(rgbImg[0] / 255.0f);
	float G = gamma(rgbImg[1] / 255.0f);
	float R = gamma(rgbImg[2] / 255.0f);
	float X = 0.412453*R + 0.357580*G + 0.180423*B;
	float Y = 0.212671*R + 0.715160*G + 0.072169*B;
	float Z = 0.019334*R + 0.119193*G + 0.950227*B;

	X /= 0.95047;
	Y /= 1.0;
	Z /= 1.08883;

	float FX = X > 0.008856f ? pow(X, 1.0f / 3.0f) : (7.787f * X + 0.137931f);
	float FY = Y > 0.008856f ? pow(Y, 1.0f / 3.0f) : (7.787f * Y + 0.137931f);
	float FZ = Z > 0.008856f ? pow(Z, 1.0f / 3.0f) : (7.787f * Z + 0.137931f);
	labImg[0] = Y > 0.008856f ? (116.0f * FY - 16.0f) : (903.3f * Y);
	labImg[1] = 500.f * (FX - FY);
	labImg[2] = 200.f * (FY - FZ);
}
void DigitDetector::computeSaliencyFT(Mat& image, Mat& binaryMap){
	assert(image.channels() == 3);
	Mat saliencyMap(image.rows, image.cols, CV_32FC1);
	Mat lab, labf;
	int h = image.rows, w = image.cols;
	labf.create(Size(w, h), CV_32FC3);
	uchar* fSrc = image.data;
	float* fLab = (float*)labf.data;
	float* fDst = (float*)saliencyMap.data;

	int stride = w * 3;
	for (int i = 0; i < h; ++i) {
		for (int j = 0; j < stride; j += 3) {
			RGBToLab(fSrc + i*stride + j, fLab + i*stride + j);
		}
	}
	float MeanL = 0, MeanA = 0, MeanB = 0;
	for (int i = 0; i < h; ++i) {
		int index = i*stride;
		for (int x = 0; x < w; ++x) {
			MeanL += fLab[index];
			MeanA += fLab[index + 1];
			MeanB += fLab[index + 2];
			index += 3;
		}
	}
	MeanL /= (w * h);
	MeanA /= (w * h);
	MeanB /= (w * h);
	GaussianBlur(labf, labf, Size(5, 5), 1);
	for (int Y = 0; Y < h; Y++)
	{
		int Index = Y * stride;
		int CurIndex = Y * w;
		for (int X = 0; X < w; X++)
		{
			fDst[CurIndex++] = (MeanL - fLab[Index]) *  \
				(MeanL - fLab[Index]) + (MeanA - fLab[Index + 1]) *  \
				(MeanA - fLab[Index + 1]) + (MeanB - fLab[Index + 2]) *  \
				(MeanB - fLab[Index + 2]);
			Index += 3;
		}
	}
	normalize(saliencyMap, saliencyMap, 0, 1, NORM_MINMAX);
	saliencyMap.convertTo(saliencyMap, CV_8UC1, 255);
	cv::threshold(saliencyMap, binaryMap, 50, 255, cv::THRESH_OTSU);
	Mat erode_kernel=cv::getStructuringElement(cv::MORPH_RECT, Size(1,1), Point(-1,-1));
	cv::erode(binaryMap, binaryMap, erode_kernel);
	//cv::bitwise_not(binaryMap, binaryMap);
}

void DigitDetector::sharpenOneChannel(Mat& image, Mat& sharpen){
	assert(image.channels()==1);
	float sharpen_data[9]={0,-2,0,-2,4,-2,0,-2,0};
	Mat sharpen_kernel(3,3,CV_32FC1,sharpen_data);
	Mat srcf(image.rows,image.cols, CV_32FC1);
	Mat sharpenf;
	image.convertTo(srcf,CV_32FC1, 1.0/255,0.0);
	cv::filter2D(srcf, sharpenf, -1, sharpen_kernel, Point(-1,-1));
	normalize(sharpenf, sharpenf, 0,1,NORM_MINMAX);
	sharpenf.convertTo(sharpen, CV_8UC1, 255.0,0.0);
}

void DigitDetector::sharpen(Mat& image, Mat& sharpenMat){
	if(image.channels()==1){
		sharpenOneChannel(image, sharpenMat);
	}
	else{
		vector<Mat> chns(3);
		vector<Mat> s_chns(3);
		cv::split(image, chns);
		for(int i=0;i<3;++i){
			sharpenOneChannel(chns[i], s_chns[i]);
		}
		cv::merge(s_chns, sharpenMat);
	}
}

void DigitDetector::binarize(Mat& image, Mat& binaryMap){
	Mat gray=image.clone();
	if(image.channels()==3){
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}
	Mat kernel=cv::getStructuringElement(cv::MORPH_RECT, Size(3,3), Point(-1,-1));
	cv::erode(gray, binaryMap, kernel);
	threshold(binaryMap, binaryMap, 50, 255, cv::THRESH_OTSU);
	//cv::bitwise_not(binaryMap, binaryMap);
}

vector<Rect> DigitDetector::getCandidateBndBoxes(Mat& binaryMap){
	vector<Rect> digitRects;
	IplImage ipl=binaryMap;
	CvMemStorage* pStorage=cvCreateMemStorage(0);
	CvSeq * pContour=NULL;
	cvFindContours(&ipl, pStorage, &pContour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for(;pContour;pContour=pContour->h_next){
		//double inner_min_area=1e4;
		CvRect bbox=cvBoundingRect(pContour,0);

		if(bbox.height>binaryMap.rows-10 or bbox.width>binaryMap.cols-10){
			cvSeqRemove(pContour, 0);
			continue;
		}
		if(1.0*bbox.width/bbox.height>2 or 1.0*bbox.height/bbox.width>2){
			cvSeqRemove(pContour, 0);
			continue;
		}
		//double real_area=fabs(cvContourArea(pContour));
		int area=bbox.width*bbox.height;
		if(area<2500){
			cvSeqRemove(pContour, 0);
			continue;
		}
		digitRects.push_back(Rect(bbox));
		cvSeqRemove(pContour, 0);
	}
	cvReleaseMemStorage(&pStorage);
	nms(digitRects);
	return digitRects;
}
