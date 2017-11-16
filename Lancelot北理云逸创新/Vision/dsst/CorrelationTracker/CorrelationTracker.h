#ifndef _CORRELATIONTRACKER_H__
#define _CORRELATIONTRACKER_H__

#include "correlationtracker_export.h"
#include <opencv2/opencv.hpp>

class CORRELATIONTRACKER_API CorrelationTracker
{
  public:
	CorrelationTracker();
	~CorrelationTracker();

	// Initialize tracker with input image <img> and init target position
	void startTrack(cv::Mat &img, cv::Rect &initRect);

	// Update tracker with new frame img, return the new target position
	cv::Rect update(cv::Mat &img);

	// Get current frame target position
	cv::Rect getPosition();

	// Get current tracker id
	int getId();
	float getPSR();

  private:
	int m_id;
	float psr;
};

#endif // _CORRELATIONTRACKER_H__
