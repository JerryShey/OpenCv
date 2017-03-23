#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>

#include <iostream>
#include <fstream>
#include <ctype.h>
#define CamaraNum 1 // computer:0, webcam:1



using namespace cv;
using namespace std;

string hot_keys =
"\n\nHot keys: \n"
"\tESC - quit the program\n"
"\tc - stop the tracking\n"
"\tb - switch to/from backprojection view\n"
"\th - show/hide object histogram\n"
"\tp - pause video\n"
"To initialize tracking, select the object with mouse\n";

const char* keys = {
	"{help h | | show help message}{@camera_number| 0 | camera number}"
};

int main(int argc, const char** argv){
	VideoCapture cap;

	cap.open(CamaraNum);

	if (!cap.isOpened()){
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		waitKey();
		return -1;
	}
	cout << hot_keys;

	Mat frame, image;
	bool paused = false, first = true, WhiteBalance = false;
	int x = 240, y = 180, width = 160, height = 120;
	Rect white = Rect(x, y, width, height);
	double Kb = 0, Kg = 0, Kr = 0;

	for (;;){
		
		if (!paused){
			cap >> frame;
			if (frame.empty()){
				cout << "Can't find camara!!" << endl;
				waitKey();
				break;
			}
		}
		

		if (WhiteBalance){
			vector<Mat> g_vChannels;
			//分離通道
			split(frame, g_vChannels);
			Mat imageBlueChannel = g_vChannels.at(0);
			Mat imageGreenChannel = g_vChannels.at(1);
			Mat imageRedChannel = g_vChannels.at(2);
			if (first){
				double imageBlueChannelAvg = 0;
				double imageGreenChannelAvg = 0;
				double imageRedChannelAvg = 0;

				//求各通道平均值
				Mat temp = Mat(imageBlueChannel, white);
				imageBlueChannelAvg = mean(temp)[0];
				temp = Mat(imageGreenChannel, white);
				imageGreenChannelAvg = mean(temp)[0];
				temp = Mat(imageRedChannel, white);
				imageRedChannelAvg = mean(temp)[0];

				//求出各通道所占增益
				double K = (imageRedChannelAvg + imageGreenChannelAvg + imageRedChannelAvg) / 3;
				Kb = K / imageBlueChannelAvg;
				Kg = K / imageGreenChannelAvg;
				Kr = K / imageRedChannelAvg;
				cout << "Kb: " << Kb << "\nKg: " << Kg << "\nKr: " << Kr;
				first = false;
			}
			//更新白平衡後的各通道BGR值
			addWeighted(imageBlueChannel, Kb, 0, 0, 0, imageBlueChannel);
			addWeighted(imageGreenChannel, Kg, 0, 0, 0, imageGreenChannel);
			addWeighted(imageRedChannel, Kr, 0, 0, 0, imageRedChannel);

			/** Rows:480, Cols:640 **/
			merge(g_vChannels, image);	//通道合併
			imshow("test", image);
		}
		else
			rectangle(frame, Point(x - 1, y - 1), Point(x + width + 1, y + height + 1), Scalar(0, 0, 255));

		imshow("video", frame);

		char c = (char)waitKey(10);
		if (c == 27)
			break;
		else if (c >= 0){
			switch (c)
			{
			case 'p':
				paused = !paused;
				break;
			case 's':
				WhiteBalance = true;
				break;
			default:
				break;
			}
		}
	}

	return 0;
}
