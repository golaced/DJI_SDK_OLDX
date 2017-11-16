#define CORRELATIONTRACKER_EXPORT

#include "CorrelationTracker.h"
#include "dsst_tracker/dsst_tracker.h"
#include <time.h>
#include <assert.h>

using namespace std;
using namespace cv;

std::vector<dlib::dsst_tracker> m_dsst_tracker_v;
int m_dsst_tracker_id = 0;

const std::string getCurrentSystemTime()
{
	time_t time_ptr;
	time(&time_ptr);
	tm *ptm = gmtime(&time_ptr);
	char date[60] = {0};
	sprintf(date, "%d-%02d-%02d--%02d:%02d:%02d",
			(int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
			(int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
	return std::string(date);
}

bool checkLisence()
{
	static const string failureTime("2018-05-01--00:00:00");
	string time_now = getCurrentSystemTime();
	if (failureTime.compare(time_now) > 0)
	{
		return true;
	}
	return false;
}

dlib::rectangle getInitPosition(const cv::Rect &r)
{
	return dlib::rectangle(r.x, r.y, r.x + r.width, r.y + r.height);
}

cv::Rect dlibRect2CVRect(const dlib::drectangle &r)
{
	return cv::Rect(r.left(), r.top(), r.width(), r.height());
}

CorrelationTracker::~CorrelationTracker()
{
}

void CorrelationTracker::startTrack(cv::Mat &img, cv::Rect &initRect)
{
	bool lisencesuccess = checkLisence();

	if (!lisencesuccess)
	{
		cout << "lisence fail!!" << endl;
		exit(-1);
		return;
	}

	m_dsst_tracker_v[m_id].init();
	CV_Assert(img.type() == CV_8UC3 || img.type() == CV_8UC1);

	cv::Mat imgGray;
	if (img.type() == CV_8UC3)
	{
		cv::cvtColor(img, imgGray, CV_8UC1);
	}
	img.copyTo(imgGray);

	dlib::cv_image<unsigned char> dlib_img(imgGray);
	m_dsst_tracker_v[m_id].start_track(dlib_img, getInitPosition(initRect));
}

cv::Rect CorrelationTracker::update(cv::Mat &img)
{
	CV_Assert(img.type() == CV_8UC3 || img.type() == CV_8UC1);

	cv::Mat imgGray;
	if (img.type() == CV_8UC3)
	{
		cv::cvtColor(img, imgGray, CV_8UC1);
	}
	img.copyTo(imgGray);

	dlib::cv_image<unsigned char> dlib_img(imgGray);
	psr = m_dsst_tracker_v[m_id].update(dlib_img);
	return getPosition();
}

cv::Rect CorrelationTracker::getPosition()
{
	return dlibRect2CVRect(m_dsst_tracker_v[m_id].get_position());
}

int CorrelationTracker::getId()
{
	return m_id;
}

float CorrelationTracker::getPSR()
{
	return psr;
}

CorrelationTracker::CorrelationTracker()
{
	if (m_dsst_tracker_v.empty())
	{
		m_dsst_tracker_id = 0;
	}
	else
	{
		m_dsst_tracker_id++;
	}

	m_id = m_dsst_tracker_id;

	dlib::dsst_tracker tracker;
	tracker.init();
	m_dsst_tracker_v.push_back(tracker);
}
