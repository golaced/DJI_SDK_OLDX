#include <fstream>
#include <sstream>
#include <thread>
#include "trace.h"
#include "imageTrans.h"
#include "my_serial.h"
#include "inifile.h"
#include <CorrelationTracker/CorrelationTracker.h>
#include <DigitDetector/DigitDetector_interface.h>
using namespace std;
using namespace cv;

bool LBtnDown = false;
int debug_screen = 0;
int flag_LX_target = 1;
extern Mat number_template[10];

bool detectNumber_digitdetector(TwoLayerNNFaster &nn, DigitDetector &detector, Mat &frame, std::vector<NumberPosition> &result)
{
	result.clear();
	vector<std::tuple<int, double, cv::Rect>> &&res = findPrintDigitAreas(nn, detector, frame);
	for (size_t i = 0; i < res.size(); ++i)
	{
		stringstream ss;
		ss << std::get<0>(res[i]);
		NumberPosition np;
		ss >> np.number_;
		np.boundRect = std::get<2>(res[i]);
		np.position_ = (np.boundRect.tl() + np.boundRect.br()) / 2;

		result.push_back(np);
	}
	std::sort(result.begin(), result.end(), SortByNumberUp);

	return true;
}

bool detectNumber_digitdetector_LED(TwoLayerNNFaster &nn, DigitDetector &detector, Mat &frame, std::vector<NumberPosition> &result)
{
	result.clear();
	vector<std::tuple<int, double, cv::Rect>> &&res = findLedDigitAreas(nn, detector, frame);
	for (size_t i = 0; i < res.size(); ++i)
	{
		stringstream ss;
		ss << std::get<0>(res[i]);
		NumberPosition np;
		ss >> np.number_;
		np.boundRect = std::get<2>(res[i]);
		np.position_ = (np.boundRect.tl() + np.boundRect.br()) / 2;

		result.push_back(np);
	}
	std::sort(result.begin(), result.end(), SortByNumberUp);

	return true;
}

void onMouse(int event, int x, int y, int, void *param)
{
	if (LBtnDown)
	{
		mouse_x = x;
		mouse_y = y;
		//uart_mouse(x, y);
		//uartSent(UART_SENT_TYPE_MOUSE);
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		LBtnDown = true;
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		LBtnDown = false;
	}
}

int char_num = 5;

int target_num = -1;
int state_num = 0;
bool isTerminal = false;
int delay_ms = 5;
bool have_target = false;
char ignore_char[10] = {66, 66, 66, 66, 66, 66, 66, 66, 66, 66};
int pre_check_count[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int camera_id = 0;
int get_camera_id(int &id)
{
	inifile::IniFile m_inifile;
	m_inifile.load(expand_user("~") + "/Lancelot/config/config.ini");
	int ret = 0;
	id = m_inifile.getIntValue("Camera", "id", ret);
	cout << "Camera id: " << id << endl;
	return ret;
}

NumberPosition number_position_send;
vector<NumberPosition> target_global(6);

#define USE_DIGITDETECTOR 0

int main(int argc, char **argv)
{
	init();

	if (get_camera_id(camera_id) != 0)
	{
		camera_id = 0;
	}
	std::thread uart_read_thread(uartReadThread);

	tesseract::TessBaseAPI tess;
	tess.Init(NULL, "eng");
	tess.SetPageSegMode(tesseract::PageSegMode::PSM_SINGLE_CHAR);

	DigitDetector detector;
	TwoLayerNNFaster nn_print(TEST);
	TwoLayerNNFaster nn_led(TEST);
	nn_print.loadParams((expand_user("~") + "/Lancelot/DigitDetector/params_hist_iter_12000.txt").c_str());
	nn_led.loadParams((expand_user("~") + "/Lancelot/DigitDetector/params_hist_led.txt").c_str());

	VideoCapture cap;

	string videoName;
	if (argc >= 2)
	{
		videoName = string(argv[1]);
	}

	CorrelationTracker tracker;

	cv::namedWindow("bar");
	int psr_threshold = 20;
	int flag_writevideo = 0;
	int flag_writing = 0;
	int flag_writevideo_src = 0;
	int flag_writing_src = 0;
	int check_count_thres = 3;
	int pre_check_count_thres = 8;

	cv::VideoWriter video_writer;
	cv::VideoWriter video_writer_src;

	cv::createTrackbar("character", "bar", &char_num, 9);
	cv::createTrackbar("LX_target", "bar", &flag_LX_target, 1);
	cv::createTrackbar("psr_threshold", "bar", &psr_threshold, 60);
	cv::createTrackbar("writevideo", "bar", &flag_writevideo, 1);
	cv::createTrackbar("writevideo_src", "bar", &flag_writevideo_src, 1);
	cv::createTrackbar("delay_ms", "bar", &delay_ms, 50);
	cv::createTrackbar("check_count_thres", "bar", &check_count_thres, 10);
	cv::createTrackbar("pre_check_count_thres", "bar", &pre_check_count_thres, 20);
	cv::createTrackbar("debug_screen", "bar", &debug_screen, 1);

#define USE_CAMERA 1
#define USE_SPACE 0

#if USE_CAMERA
	cap.open(camera_id);
	waitKey(1000);

	if (!cap.isOpened())
	{
		cout << "camera open failed!" << endl;
		return -1;
	}

#else
	cap.open(videoName);

	waitKey(1000);

#endif

	Mat src, src_temp, src_save;
	cap.set(CV_CAP_PROP_FPS, 60);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	cap >> src;
	imshow("cap", src);
	setMouseCallback("cap", onMouse, NULL); //mouse interface

#if USE_CAMERA
#if USE_SPACE
	while (true)
	{
		cap >> src_temp;
		warpFfine(src_temp, src, 180);
		imshow("cap", src);

		if (LBtnDown)
		{
			imshow("cap", src);
			cout << "LBTN:" << LBtnDown << endl;
			uartSent(UART_SENT_TYPE_MOUSE);
			waitKey(10);
			continue;
		}
		//uart_mouse(0, 0);
		uartSent(UART_SENT_TYPE_MOUSE);

		char c = waitKey(10);

		if (c == 32)
		{
			break;
		}
		if (c == 27)
		{
			return 0;
		}
	}
#endif

#endif

	if (src.data == NULL)
	{
		cout << "camera open failed!" << endl;
		return -1;
	}

	TickMeter tm;
	tm.getTimeMilli();

	int frame = 0;
	int frame_check = 0;

	Size _size(300, 400);

	vector<NumberPosition> result;

	float tracker_psr = 0.0;

	bool start_track = false;

	Mat imgGray;
	cvtColor(src, imgGray, CV_BGR2GRAY);

	//int count_frame = 0;
	char temp_text[50];
	int character_to_recog = 0;
	Point pt_src_center(src.cols / 2, src.rows / 2);
	int check_count = 0;
	char check_character = 0;
	char check_character_temp = 0;
	int nullcount = 0;

	while (true)
	{
		frame++;
// log_out << "frame: " << frame << endl;

#if USE_CAMERA
		cap >> src_temp;
		warpFfine(src_temp, src, 180);
		src.copyTo(src_save);
#else
		cap >> src;
//cap.set(CV_CAP_PROP_POS_FRAMES,5000);
#endif

		if (LBtnDown)
		{
			imshow("cap", src);
			cout << "LBTN:" << LBtnDown << endl;
			uartSent(UART_SENT_TYPE_MOUSE);
			waitKey(10);
			continue;
		}

		cvtColor(src, imgGray, CV_BGR2GRAY);

		if (src.data == NULL)
		{
			cout << "src.data = NULL...try again..." << endl;
			nullcount++;
			if (nullcount > 10)
			{
				return -1;
			}
			continue;
		}

		target_global.clear();
		target_global.resize(6, NumberPosition());

		if (flag_LX_target == 1)
		{
			if (state_num == SD_FLY_TARGET)
			{
				check_count = 0;
				have_target = false;
				target_num = -1;
				check_character = 0;
				check_character_temp = 0;
				for (size_t i = 0; i < 10; i++)
				{
					pre_check_count[i] = 0;
				}
				//continue;
			}
			if (state_num == SD_SCAN_9 || state_num == SD_SCAN_10 || state_num == SD_SCAN_11)
			{
#if USE_DIGITDETECTOR
				detectNumber_digitdetector(nn_print, detector, src, target_global);
#else
				detectNumber(src, tess, target_global);
#endif
				for (size_t i = 0; i < target_global.size(); i++)
				{
					putText(src, target_global[i].number_, target_global[i].position_,
							FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 2);
				}
				for (size_t i = target_global.size(); i < 6; i++)
				{
					target_global.push_back(NumberPosition());
				}
			}
			//state_num == SD_CHECK_TARGET;
			if (state_num == SD_CHECK_TARGET)
			{
				frame_check++;
				//				for (size_t i = 0; i < 10; i++)
				//				{
				//					cout << pre_check_count[i] << "  ";
				//				}
				//				cout<<frame_check<<endl;
				if (frame_check % 200 == 0)
				{
					for (size_t i = 0; i < 10; i++)
					{
						pre_check_count[i] = 0;
					}
				}
#if USE_DIGITDETECTOR
				detectNumber_digitdetector_LED(nn_print, detector, src, result);
#else
				detectNumber_LED(src, tess, result);
#endif
				bool checked = false;
				bool ignored = false;

				for (size_t i = 0; i < result.size(); i++)
				{
					ignored = false;
					for (size_t j = 0; j < 10; j++)
					{
						if (result[i].number_[0] == ignore_char[j] + 48)
						{
							ignored = true;
							break;
						}
					}
					if (ignored)
					{
						putText(src, result[i].number_, result[i].position_,
								FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 0), 2);
						continue;
					}

					putText(src, result[i].number_, result[i].position_,
							FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 2);

					pre_check_count[result[i].number_[0] - 48]++;

					if (check_character == result[i].number_[0])
					{
						check_character = result[i].number_[0];
						checked = true;
					}
				}
				if (checked)
				{
					check_count++;
				}
				else
				{
					check_count = 0;

					for (size_t i = 0; i < result.size(); i++)
					{
						ignored = false;
						for (size_t j = 0; j < 10; j++)
						{
							if (result[i].number_[0] == ignore_char[j] + 48)
							{
								ignored = true;
								break;
							}
						}
						if (!ignored)
						{
							check_character = result[i].number_[0];
							break;
						}
					}
				}
				//				if (result.size() > 0)
				//				{
				//					if (check_character == result[0].number_[0])
				//					{
				//						check_count++;
				//					}
				//					else
				//					{
				//						check_count = 0;
				//					}
				//
				//					check_character = result[0].number_[0];
				//				}
				if (check_count >= check_count_thres)
				{
					have_target = true;
					target_num = check_character - 48;
				}
				for (size_t target_index = 0; target_index < 10; target_index++)
				{
					if (pre_check_count[target_index] >= pre_check_count_thres)
					{
						have_target = true;
						target_num = target_index;
						break;
					}
				}

				cout << "send:" << target_num << endl;

				uartSent(UART_SENT_TYPE_TARGET);

				//continue;
			}
			else
			{
				check_count = 0;
				for (size_t i = 0; i < 10; i++)
				{
					pre_check_count[i] = 0;
				}
			}

			character_to_recog = target_num;
		}
		else
		{
			have_target = true;
			character_to_recog = char_num;
		}

		if ((flag_LX_target == 0) || (state_num >= SD_HOLD)) //fly to workspace, detect, track, print and so on
		{
			if (start_track)
			{
#if COUT_TIME
				tm.reset();
				tm.start();
				tracker.update(imgGray);
				tm.stop();
				cout << "tracker.update: " << tm.getTimeMilli() << " ms" << endl;
#else
				tracker.update(imgGray);
#endif
				tracker_psr = tracker.getPSR() * 2;
				static int cnt_loss_track = 0, cnt_loss_track5 = 0;
				if (tracker_psr < 12)
					cnt_loss_track++;
				if (tracker_psr < 7)
					cnt_loss_track5++;
				if (cnt_loss_track > 20 || cnt_loss_track5 > 2)
				{
					cnt_loss_track = 0;
					cnt_loss_track5 = 0;
					start_track = false;
				}

				if (tracker_psr < psr_threshold || frame % 50 == 0)
				{
					start_track = false;
				}

				Rect rect_temp = tracker.getPosition();
				number_position_send.position_ = Point(rect_temp.x + rect_temp.width / 2,
													   rect_temp.y + rect_temp.height / 2);
				cv::rectangle(src, rect_temp, CV_RGB(255, 0, 0), 2, 8, 0);
				putText(src, number_position_send.number_,
						number_position_send.position_, FONT_HERSHEY_SIMPLEX, 1,
						CV_RGB(255, 0, 0), 2);
				sprintf(temp_text, "psr=%d", int(tracker_psr));
				putText(src, temp_text, Point(10, 20), FONT_HERSHEY_SIMPLEX,
						0.6, CV_RGB(255, 0, 0), 2);
				cv::circle(src,
						   Point(rect_temp.x + rect_temp.width / 2,
								 rect_temp.y + rect_temp.height / 2),
						   2, CV_RGB(255, 0, 0), 2, 8, 0);
			}
			else
			{
				number_position_send.init();
				target_global.clear();
#if USE_DIGITDETECTOR
				detectNumber_digitdetector(nn_print, detector, src, result);
#else
				detectNumber(src, tess, result);
#endif
				for (size_t i = 0; i < result.size(); i++)
				{
					target_global.push_back(result[i]);
				}
				for (size_t i = target_global.size(); i < 6; i++)
				{
					target_global.push_back(NumberPosition());
				}

				for (size_t i = 0; i < result.size(); i++)
				{
					//cout<<result[i].number_<<endl;
					if (result[i].number_[0] == character_to_recog + 48)
					{
						number_position_send.number_ = result[i].number_;
						number_position_send.position_ = result[i].position_;
						number_position_send.boundRect = result[i].boundRect;
#if COUT_TIME
						tm.reset();
						tm.start();
						tracker.startTrack(imgGray, number_position_send.boundRect);
						tm.stop();
						cout << "tracker.startTrack: " << tm.getTimeMilli() << " ms" << endl;
#else
						tracker.startTrack(imgGray, number_position_send.boundRect);
#endif
						start_track = true;
					}
					putText(src, result[i].number_, result[i].position_,
							FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 2);
				}
			}
		}

		else
		{
			number_position_send.init();
			start_track = false;
		}

		if (number_position_send.position_.x > 0)
		{
			sprintf(temp_text, "position=%d,%d",
					int(number_position_send.position_.x),
					number_position_send.position_.y);
			putText(src, temp_text, Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.6,
					CV_RGB(255, 0, 0), 2);
		}

		drawImage(src);

		uartSent(UART_SENT_TYPE_SCAN);
		uartSent(UART_SENT_TYPE_CHARACTER);
		uartSent(UART_SENT_TYPE_TARGET);
		uartSent(UART_SENT_TYPE_MOUSE);

		imshow("cap", src);

		if (flag_writing == 1)
		{
			video_writer << src;
			if (flag_writevideo == 0)
			{
				flag_writing = 0;
				video_writer.release();
			}
		}
		else
		{
			if (flag_writevideo == 1)
			{
				startWriteVideo(video_writer);
				flag_writing = 1;
			}
		}

		if (flag_writing_src == 1)
		{
			video_writer_src << src_save;
			if (flag_writevideo_src == 0)
			{
				flag_writing_src = 0;
				video_writer_src.release();
			}
		}
		else
		{
			if (flag_writevideo_src == 1)
			{
				startWriteVideo(video_writer_src);
				flag_writing_src = 1;
			}
		}

		char c = waitKey(1);
		if (c == 27)
		{
			break;
		}
	}

	// log_out.close();
	if (video_writer.isOpened())
	{
		video_writer.release();
	}
	if (video_writer_src.isOpened())
	{
		video_writer_src.release();
	}

	isTerminal = true;
	uart_read_thread.join();

	return 0;
}
