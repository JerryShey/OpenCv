#include "opencv2/video/tracking.hpp"
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
Mat image, templ, myGoal;

int vmin = 10, vmax = 180, smin = 100, hmax = 180, hmin = 0, smax = 256;
void contourTest(Mat);
void DataAnalysis(Mat);
void yungyung();
void setGoal(RotatedRect);
void charRecongnizq(Mat);

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
bool drowCircle = true;

bool set(int x, int y, int width, int height){
	origin = Point(x, y);
	selection = Rect(x, y, width, height);
	selection &= Rect(0, 0, image.cols, image.rows);

	trackObject = -1;
	return true;
}

string hot_keys =
	"\n\nHot keys: \n"
	"\tESC - quit the program\n"
	"\tb - switch to/from backprojection view\n"
	"\th - show/hide object histogram\n"
	"\tp - pause video\n"
	"\ts - set your goal with the red rectangle\n"
	"\tw - using white balance\n";


int main( int argc, const char** argv ){
    VideoCapture cap;
    Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
	Mat colorMat;
	int d1num = 1, d2num = 1, e1num = 1, e2num = 1;

	cap.open(CamaraNum);

    if( !cap.isOpened() ){
        cout << "***Could not initialize capturing...***\n";
        cout << "Current parameter's value: \n";
		waitKey();
        return -1;
    }
    cout << hot_keys;
    //namedWindow( "Histogram", 0 );
    namedWindow( "CamShift Demo", 0 );
	namedWindow("controller", 0);
	createTrackbar("E1", "controller", &d1num, 10, 0);
	createTrackbar("D1", "controller", &e1num, 10, 0);
	createTrackbar("D2", "controller", &d2num, 10, 0);
	createTrackbar("E2", "controller", &e2num, 10, 0);
	createTrackbar("Smin", "controller", &smin, 255, 0);
	createTrackbar("Smax", "controller", &smax, 255, 0);
	createTrackbar("Vmin", "controller", &vmin, 255, 0);
	createTrackbar("Vmax", "controller", &vmax, 255, 0);

	Mat frame, hsv, hue, mask, hist, backproj; //, histimg = Mat::zeros(200, 320, CV_8UC3)
	int x = 240, y = 180, width = 160, height = 120;
	bool paused = false, select = false, first = true, WhiteBalance = false;
	Rect white = Rect(x, y, width, height);
	double Kb = 0, Kg = 0, Kr = 0;

    for(;;){
		drowCircle = true;
        if( !paused ){
            cap >> frame;
			if (frame.empty()){
				cout << "Can't find camara!!" << endl;
				waitKey();
				break;
			}
        }
		
        frame.copyTo(image);
		/** Rows:480, Cols:640 **/
        if( !paused ){
			if (WhiteBalance){
				vector<Mat> g_vChannels;
				//分離通道
				split(image, g_vChannels);
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

				merge(g_vChannels, image);	//通道合併
			}
            cvtColor(image, hsv, COLOR_BGR2HSV);
			
			
			if (trackObject){

				inRange(hsv, Scalar(hmin, smin, vmin),
					Scalar(hmax, smax, vmax), mask);
				//inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);
				

				int ch[] = { 0, 0 };
				hue.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue, 1, ch, 1);

				if (trackObject < 0){
					templ = Mat(image, selection);
					imwrite("templ.jpg", templ);
					//imshow("templ", templ);

					// Object has been selected by user, set up CAMShift search properties once
					Mat roi(hue, selection), maskroi(mask, selection);
					calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
					normalize(hist, hist, 0, 255, NORM_MINMAX);

					trackWindow = selection;
					trackObject = 1; // Don't set up again, unless user selects new ROI

					/*histimg = Scalar::all(0);
					int binW = histimg.cols / hsize;
					Mat buf(1, hsize, CV_8UC3);
					for (int i = 0; i < hsize; i++)
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
					cvtColor(buf, buf, COLOR_HSV2BGR);

					for (int i = 0; i < hsize; i++){
						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows / 255);
						rectangle(histimg, Point(i*binW, histimg.rows),
							Point((i + 1)*binW, histimg.rows - val),
							Scalar(buf.at<Vec3b>(i)), -1, 8);
					}*/
				}

				//erode 侵蝕物體 //dilate 物體膨脹
				for (int i = 0; i < d2num; i++)
					dilate(hue, hue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				for (int i = 0; i < e2num; i++)
					erode(hue, hue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				//getStructuringElement(MORPH_ELLIPSE, Size(5, 5))

				for (int i = 0; i < e1num; i++)
					erode(hue, hue, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
				for (int i = 0; i < d1num; i++)
					dilate(hue, hue, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));

				
				
				// Perform CAMShift
				calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
				backproj &= mask;
				

				RotatedRect trackBox;
				try{
					trackBox = CamShift(backproj, trackWindow, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));
					//yungyung();
					setGoal(trackBox);
				}	
				catch (Exception e){
					trackWindow = Rect(50, 128, image.cols*0.8, image.rows*0.8);
					//destroyWindow("myGoal");
					imshow("CamShift Demo", image);
					//imshow("Histogram", histimg);
					drowCircle = false;
				}
				if (trackBox.size.height < 50 || trackBox.size.width < 50){
					trackWindow = Rect(50, 128, image.cols*0.8, image.rows*0.8);
					//destroyWindow("myGoal");
					imshow("CamShift Demo", image);
					//imshow("Histogram", histimg);
					drowCircle = false;
				}

				if (backprojMode){
					cvtColor(backproj, image, COLOR_GRAY2BGR);
					//contourTest(backproj);
				}

				if (drowCircle)
					ellipse( image, trackBox, Scalar(0,0,255), 3, 8);
            }
			else
				rectangle(image, Point(x-1, y-1), Point(x+width+1, y+height+1), Scalar(0, 0, 255));
        }
        else if( trackObject < 0 )
            paused = false;

        imshow( "CamShift Demo", image );
        //imshow( "Histogram", histimg );
		
		int i, j;
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
		else if (c >= 0){
			switch (c){
			case 'b':
				backprojMode = !backprojMode;
				break;
			case 'h':
				showHist = !showHist;
				if (!showHist)
					destroyWindow("Histogram");
				else
					namedWindow("Histogram", 1);
				break;
			case 'p':
				paused = !paused;
				break;
			case 's':
				if (select){
					trackObject = 0;
					destroyWindow("templ");
					//destroyWindow("myGoal");
					//histimg = Scalar::all(0);
				}
				else{
					set(x, y, width, height);
					cvtColor(image, hsv, COLOR_BGR2HSV);
					hsv.copyTo(colorMat);

					DataAnalysis(colorMat);
				}
				select = !select;
				break;
			case 'w':
				WhiteBalance = true;
				break;
			default:
				break;
			}
		}
		
    }

    return 0;
}

void setGoal(RotatedRect lulu){
	Rect tongtong;
	double x, y, width, height;
	x = lulu.center.x - lulu.size.width / 2;
	if (x < 0){
		width = lulu.size.width + x;
		x = 0;
	}
	else if ((x + lulu.size.width) > image.cols){
		width = image.cols - x - 1;
		if (width > image.cols){
			width = image.cols - 1;
		}
	}
	else{
		width = lulu.size.width;
	}
	y = lulu.center.y - lulu.size.height / 2;
	if (y < 0){
		height = lulu.size.height / 2 + y;
		y = 0;
	}
	else if ((y + lulu.size.height) > image.rows){
		height = image.rows - y - 1;
		if (height > image.rows){
			height = image.rows - 1;
		}
	}
	else{
		height = lulu.size.height;
	}
	//cout << image.rows << ", " << image.cols << endl;
	//cout << "lulu:width, height" << lulu.size.width << ", " << lulu.size.height << endl;
	//cout << "tongtong: width, height" << width << ", " << height << endl;

	tongtong = Rect(x, y, floor(width), floor(height));
	//cout << tongtong << endl;
	myGoal = Mat(image, tongtong);
	//imshow("myGoal", myGoal);
	templ = imread("templ.jpg");
}

void DataAnalysis(Mat mat){
	//int x = 240, y = 180, width = 160, height = 120;
	Vec3b co;
	fstream file;
	int n = 0;
	file.open("C:\\Users\\user\\Desktop\\Data Analysis.txt", ios::out);
	float totalH = 0.0, totalV = 0.0, totalS = 0.0;
	float totalH2 = 0.0, totalV2 = 0.0, totalS2 = 0.0;
	float sigmaH = 0.0, sigmaS = 0.0, sigmaV = 0.0;
	for (int i = 240; i < 400; i += 3)
		for (int j = 180; j < 300; j += 1){
			co = mat.at<Vec3b>(j, i);
			//cout << ++n << ". " << co << "\tcols:" << i << ", rows:" << j << endl;
			file << n+1 << ". "<< co << endl;
			int H = co.val[0];
			int S = co.val[1];
			int V = co.val[2];
			totalH += H;
			totalS += S;
			totalV += V;
			totalH2 += H * H;
			totalS2 += S * S;
			totalV2 += V * V;
			n++;
		}
	sigmaH = sqrt(totalH2 / n - (totalH * totalH) / (n*n));
	sigmaS = sqrt(totalS2 / n - (totalS * totalS) / (n*n));
	sigmaV = sqrt(totalV2 / n - (totalV * totalV) / (n*n));

	vmax = totalV / n + 5 * sigmaV;
	vmin = totalV / n - 5 * sigmaV;
	
	if (vmax > 255 || vmax < 0) vmax = 256;
	if (vmin < 1) vmin = 1;

	smax = totalS / n + 5 * sigmaS;
	smin = totalS / n - 5 * sigmaS;

	if (smax > 255 || vmax < 0) smax = 256;
	if (smin < 1) smin = 1;

	hmax = totalH / n + sigmaH * 5;
	hmin = totalH / n - sigmaH * 5;
	
	if (hmax > 180) hmax = 180;
	if (hmin < 0) hmin = 0;

	file <<"色相H\t標準差：" << sigmaH << "平均數：" << totalH / n << endl;
	file << "飽和度S\t標準差：" << sigmaS << "平均數：" << totalS / n << endl;
	file << "明度V\t標準差：" << sigmaV << "平均數：" << totalV / n << endl;
	cout << "Success" << endl;
	file.close();
}

void contourTest(Mat backproj){
	int thresh = 100;
	RNG rng(12345);
	Mat canny_output;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat drawing;
	vector<Point> lines[50];

	backproj.copyTo(drawing);
	cvtColor(drawing, drawing, COLOR_GRAY2BGR);

	/// Detect edges using canny
	Canny(backproj, canny_output, thresh, thresh * 2, 3);

	/// Find contours
	/*HoughLinesP(canny_output, hierarchy, 1, CV_PI / 180, 30);
	for (int i = 0; i<hierarchy.size(); i++){
		line(drawing, Point(hierarchy[i][0], hierarchy[i][1]), Point(hierarchy[i][2], hierarchy[i][3]), Scalar(255, 255, 255), 3);
	}*/
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	for (int i = 0; i< contours.size(); i++)
	{
		if (contours[i].size() <= 7)
			continue;
		approxPolyDP(contours[i], contours[i], 5, false);
		drawContours(drawing, contours, i, Scalar(255, 255, 255), 2, 8, hierarchy, 0, Point());
	}
	

	/// Detect edges using canny
	Canny(drawing, canny_output, thresh, thresh * 2, 3);

	
	
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	for (int i = 0; i< contours.size(); i++)
	{
		if (contours[i].size() <= 15)
			continue;
		approxPolyDP(contours[i], contours[i], 5, false);
		drawContours(drawing, contours, i, Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point());
	}
		/*for (int j = 0; j < 50; j++){
			if (lines[j].empty()){
				lines[j].push_back(contours[i][0]);
				lines[j].push_back(contours[i][contours[i].size() - 1]);
				break;
			}
			else{
				int deltaX = lines[j].at(lines[j].size() - 1).x - contours[i][0].x;
				int deltaY = lines[j].at(lines[j].size() - 1).y - contours[i][0].y;
				if (deltaX < 0)	deltaX *= -1;
				if (deltaY < 0)	deltaY *= -1;
				if (deltaX == 0 && deltaY == 0)
					break;
				if (deltaX < 5 && deltaY < 5){
					lines[j].push_back(contours[i][0]);
					lines[j].push_back(contours[i][contours[i].size() - 1]);
					break;
				}
			}
		}*/


	imshow("Contours", drawing);
}