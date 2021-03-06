/*****************************
Copyright 2011 Rafael Mu��oz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list
of conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Mu��oz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Mu��oz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Mu��oz Salinas.
********************************/

#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <map>
#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"
#include "my_serial.h"
#include <fstream>
#include "ColorDetect.h"

extern std::vector<MarkerWorld> CoordinateTable;

using namespace std;
using namespace cv;
using namespace aruco;

#define TRANS_WORLD 1
#define WRITE_VIDEO  1
#define SIZE_METER 0
#define USE_PIXLE_LAND 1

int video_num = 0;
std::ifstream video_num_read;
std::ofstream video_num_write;
std::string video_num_path(
        "/home/odroid/workspace/video/video_num.txt");

void startWriteVideo(std::ifstream &video_num_read,
        cv::VideoWriter &video_writer, cv::Size m_size, int fps)
{
    video_num_read.open(video_num_path.c_str());
    video_num_read >> video_num;
    video_num_read.close();

    cout << "video_num:"<<video_num << endl;

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

    video_writer.open(
            "/home/odroid/workspace/video/" + video_name,
            CV_FOURCC('D', 'I', 'V', 'X'), fps, m_size);

}


void warpFfine(cv::Mat &inputIm, cv::Mat &tempImg, float angle)
{
    CV_Assert(!inputIm.empty());
    Mat inputImg;
    inputIm.copyTo(inputImg);
    float radian = (float) (angle / 180.0 * CV_PI);
    int uniSize = (int) (max(inputImg.cols, inputImg.rows) * 1.414);
    int dx = (int) (uniSize - inputImg.cols) / 2;
    int dy = (int) (uniSize - inputImg.rows) / 2;
    copyMakeBorder(inputImg, tempImg, dy, dy, dx, dx, BORDER_CONSTANT);
    Point2f center((float) (tempImg.cols / 2), (float) (tempImg.rows / 2));
    Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);
    warpAffine(tempImg, tempImg, affine_matrix, tempImg.size());
    float sinVal = fabs(sin(radian));
    float cosVal = fabs(cos(radian));
    Size targetSize((int) (inputImg.cols * cosVal + inputImg.rows * sinVal),
            (int) (inputImg.cols * sinVal + inputImg.rows * cosVal));
    int x = (tempImg.cols - targetSize.width) / 2;
    int y = (tempImg.rows - targetSize.height) / 2;
    Rect rect(x, y, targetSize.width, targetSize.height);
    tempImg = Mat(tempImg, rect);
}

bool loadConfig(std::string configfile_path, std::map<int, MarkerConfig> &markermap)
{
	std::ifstream configFileIn;
	configFileIn.open(configfile_path);

	if (!configFileIn.is_open())
	{
		return false;
	}

	char temp[255];
	int id = -1;
    float boardsize = -1;
    float x = -1;
    float y = -1;
    float z = -1;

	while (!configFileIn.eof())
	{
		configFileIn.getline(temp, 255);
        sscanf(temp, "%d %f %f %f %f", &id, &boardsize, &x, &y, &z);

		if (id>0)
		{
#if SIZE_METER
            markermap.insert(make_pair(id, MarkerConfig(id, boardsize/100, x/100, y/100, z/100)));
#else
            markermap.insert(make_pair(id, MarkerConfig(id, boardsize, x, y, z)));
#endif

		}
	}

	return true;
}

cv::Point3f coordinate_camera(0, 0, 0);
Attitude atti_camera;
std::vector< aruco::Marker > Markers;
std::map<int, MarkerConfig> markermap;
std::vector<cv::Rect> objectRects;
std::vector<cv::Point> objectCenters;
std::vector<int> objectLocate;
cv::Scalar markerorigin_img(0,0);
cv::Size m_size(0,0);
cv::Point image_center(0,0);
int main()
{
	try
	{
#define CAP_WIDTH_320 1

#if CAP_WIDTH_320
        string cameraParamFileName("/home/odroid/workspace/QT/QRLand/PS3_320.yml");
        m_size = cv::Size(320, 240);
        image_center = Point(160,120);
#else
        string cameraParamFileName("/home/odroid/workspace/QT/QRLand/PS3_640.yml");
        m_size = cv::Size(640, 480);
        image_center = Point(320,240);
#endif
		string imagename("SizeQR.png");
		//string imagename("3.bmp");

        string configpath("/home/odroid/workspace/QT/QRLand/config.ini");
		bool bloadOK = loadConfig(configpath, markermap);
		if (!bloadOK)
		{
			cout << "markers config load failed!" << endl;
			return -1;
		}

		initCoordinateTable(CoordinateTable);

		aruco::CameraParameters CamParam;
		MarkerDetector MDetector;
        ColorDetect colorDetector;

		// read the input image
		cv::Mat InImage;
        cv::Mat SrcImage;
        cv::Mat src_temp;
		// try opening first as video
		VideoCapture cap(0);

		cap >> InImage;
		resize(InImage, InImage, m_size);

		//read camera parameters if specifed
		CamParam.readFromXMLFile(cameraParamFileName);

		cout << CamParam.CameraMatrix << endl;
		cout << CamParam.Distorsion << endl;
		cout << CamParam.CamSize << endl;

		cv::String windowTitle("thes");
		cv::namedWindow(windowTitle, 1);

		int p1 = 7;
		int p2 = 7;
		int t_p_range = 2;
		createTrackbar("p1", "thes", &p1, 101);
		createTrackbar("p2", "thes", &p2, 50);
		createTrackbar("range", "thes", &t_p_range, 31);

		ostringstream ostr_pos;
		ostringstream ostr_angle;

#if WRITE_VIDEO

        int fps = 30;
        cv::VideoWriter video_writer;
        cv::VideoWriter videowriter_process;
        startWriteVideo(video_num_read, video_writer, m_size, fps);
        startWriteVideo(video_num_read, videowriter_process, m_size, fps);

#endif

		while (true)
		{
			p1 = p1 / 2 * 2 + 1;
			p2 = p2 / 2 * 2 + 1;
			MDetector.setThresholdParamRange(t_p_range);
			MDetector.setThresholdParams(p1, p2);

            cap >> src_temp;
            warpFfine(src_temp, InImage, 180);

			resize(InImage, InImage, m_size);
            InImage.copyTo(SrcImage);

			// Ok, let's detect
			MDetector.detect(InImage, Markers, CamParam);

            float markersize = 0;
            for (auto it = Markers.begin(); it != Markers.end(); )
            {
                //cout<<it->getPerimeter()<<endl;
                if ((it->getPerimeter())<20)
                {
                    Markers.erase(it);
                    continue;
                }

                markersize = ((markermap.find(it->id))->second).boardsize;
                if(markersize<=0)
                {
                    Markers.erase(it);
                }
                else
                {
                    it->draw(InImage, Scalar(0, 0, 255), 2);
                    it->calculateExtrinsics(markersize, CamParam, false);
                    it++;
                }

            }

			getCameraPosWithMarkers(Markers, coordinate_camera,atti_camera, 0);
#if USE_PIXLE_LAND
            markerorigin_img = getLandCenter(Markers);

            Point center_pt(int(markerorigin_img.val[0]), int(markerorigin_img.val[1]));
            cv::circle(InImage,center_pt, 5, Scalar(255, 0, 255), -1);
	    ostr_pos.clear();
	    ostr_pos.str("");
	    ostr_pos << center_pt;

	    putText(InImage, ostr_pos.str(), center_pt+Point(10,10), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

	    ostr_pos.clear();
	    ostr_pos.str("");
	    ostr_pos << "dis="<<Point(int(markerorigin_img.val[2]),int(markerorigin_img.val[3]));

	    putText(InImage, ostr_pos.str(), Point(180, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);
#endif
            colorDetector.setImage(SrcImage);
            objectLocate = colorDetector.getColorRegionLocate(ColorDetect::RED);

			serialSent();

			ostr_pos.clear();
			ostr_pos.str("");
#if SIZE_METER
            coordinate_camera.x*=100;
            coordinate_camera.y*=100;
            coordinate_camera.z*=100;
#endif
            ostr_pos << Point3i((int)coordinate_camera.x,(int)coordinate_camera.y ,(int)coordinate_camera.z);

	    putText(InImage, ostr_pos.str(), Point(15, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

            objectRects = colorDetector.getRects();
            objectCenters = colorDetector.getRectCenters();

            for(size_t i = 0;i<objectRects.size();i++)
            {
                cv::rectangle(InImage,objectRects[i],Scalar(255,0,0),2);
                cv::circle(InImage,objectCenters[i], 3, Scalar(255, 0, 0), -1);
            }

			// show input with augmented information
	    cv::imshow("in", InImage);
#if WRITE_VIDEO
            video_writer << SrcImage;
            videowriter_process<<InImage;
#endif
			// show also the internal image resulting from the threshold operation
            cv::imshow("thes", MDetector.getThresholdedImage());

            char c_key = cv::waitKey(1);
	    if (c_key == 27) // wait for key to be pressed
	    {
		break;
	    }
	}

#if WRITE_VIDEO
        video_writer.release();
		videowriter_process.release();
#endif
		return 0;
}
	catch (std::exception &ex)
	{
		cout << "Exception :" << ex.what() << endl;
	}
}
