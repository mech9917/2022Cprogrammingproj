/*
* Documentation Date: 23.May, 2022
* Author : Heesung Kim
* Senior in Department of Mechanical Engineering,
* Sogang University, Republic of Korea
*/

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "SerialClass.h"

using namespace cv;
using namespace std;

int threshold1 = 30;
int rho_unit = 1;
int theta_degree_unit = 1;
int ptr_votes_thres = 20;
double toRadian = (CV_PI / 180);
double toDegree = 1 / toRadian;

Vec3b lower_blue1, upper_blue1, lower_blue2, upper_blue2, lower_blue3, upper_blue3;
Mat img_color, img_color2hough;

void mouse_callback(int event, int x, int y, int flags, void* param)
{
	/*
	* If left button of mouse is clicked on original image,
	* get the BGR color coordinate of the point,
	* transform into HSV coordinate, set colormask,
	* and display current HSV value, color threshold, and mask values.
	*/
	if (event == EVENT_LBUTTONDOWN)
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x);

		Mat bgr_color = Mat(1, 1, CV_8UC3, color_pixel);

		Mat hsv_color;
		cvtColor(bgr_color, hsv_color, COLOR_BGR2HSV);

		int hue = hsv_color.at<Vec3b>(0, 0)[0];
		int saturation = hsv_color.at<Vec3b>(0, 0)[1];
		int value = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "hue = " << hue << endl;
		cout << "saturation = " << saturation << endl;
		cout << "value = " << value << endl;

		if (hue < 10)
		{
			cout << "case 1" << endl;
			lower_blue1 = Vec3b(hue - 10 + 180, threshold1, threshold1);
			upper_blue1 = Vec3b(180, 255, 255);
			lower_blue2 = Vec3b(0, threshold1, threshold1);
			upper_blue2 = Vec3b(hue, 255, 255);
			lower_blue3 = Vec3b(hue, threshold1, threshold1);
			upper_blue3 = Vec3b(hue + 10, 255, 255);
		}
		else if (hue > 170)
		{
			cout << "case 2" << endl;
			lower_blue1 = Vec3b(hue, threshold1, threshold1);
			upper_blue1 = Vec3b(180, 255, 255);
			lower_blue2 = Vec3b(0, threshold1, threshold1);
			upper_blue2 = Vec3b(hue + 10 - 180, 255, 255);
			lower_blue3 = Vec3b(hue - 10, threshold1, threshold1);
			upper_blue3 = Vec3b(hue, 255, 255);
		}
		else
		{
			cout << "case 3" << endl;
			lower_blue1 = Vec3b(hue, threshold1, threshold1);
			upper_blue1 = Vec3b(hue + 10, 255, 255);
			lower_blue2 = Vec3b(hue - 10, threshold1, threshold1);
			upper_blue2 = Vec3b(hue, 255, 255);
			lower_blue3 = Vec3b(hue - 10, threshold1, threshold1);
			upper_blue3 = Vec3b(hue, 255, 255);
		}
		cout << "hue = " << hue << endl;
		cout << "threshold = " << threshold1 << endl;
		cout << "#1 = " << lower_blue1 << "~" << upper_blue1 << endl;
		cout << "#2 = " << lower_blue2 << "~" << upper_blue2 << endl;
		cout << "#3 = " << lower_blue3 << "~" << upper_blue3 << endl;
	}
}

double drawHoughLines(vector<Vec2f> lines)
{
	/*
	* Define one point (x0,y0) in a line
	* To draw infinite length, set large value of line length
	* Draw line at image, display rho, theta, endpoints
	* Return angle
	*/
	for (int i = 0; i < lines.size(); i++)
	{
		double rho = lines[i][0], theta = lines[i][1];
		double a = cos(theta);
		double b = sin(theta);
		double angle = lines[i][1] * (180 / CV_PI);
		if (angle > 90)
		{
			angle = angle - 180;
		}

		double x0 = rho * a, y0 = rho * b;
		double dL = 1000;
		Point pt1, pt2;
		pt1.x = x0 - dL * b;
		pt1.y = y0 + dL * a;
		pt2.x = x0 + dL * b;
		pt2.y = y0 - dL * a;

		 cout << "line " << i + 1
		      << "->> rho : " << lines[i][0] << "px, \t"
		      << " theta : " << angle
		      << "\t Point1(x,y) : " << pt1.x << "\t ," << pt1.y
		      << "\t Point2(x,y) : " << pt2.x << "\t ," << pt2.y
		      << endl;
		
		line(img_color2hough, pt1, pt2, Scalar(0, 200, 200), 3, LINE_AA);
		return angle;
	}
};

int TX_DATA(Serial* ser, char tilt, double angle) {
	/*
	* Determine which character to send in Arduino.
	* Steering angle or front wheel will change(refer to .ino code)
	*/
	if (tilt == 'L')
	{
		ser->WriteData("L", 1);
		Sleep(200);
		cout << "Strong Left" << "\t\t HoughLine Angle : " << angle << endl;
	}
	else if (tilt == 'l')
	{
		ser->WriteData("l", 1);
		Sleep(200);
		cout << "Weak Left" << "\t\t HoughLine Angle : " << angle << endl;
	}
	else if (tilt == 'N')
	{
		ser->WriteData("N", 1);
		Sleep(200);
		cout << "Neutral" << "\t\t HoughLine Angle : " << angle << endl;
	}
	else if (tilt == 'r')
	{
		ser->WriteData("r", 1);
		Sleep(200);
		cout << "Weak Right" << "\t\t HoughLine Angle : " << angle << endl;
	}
	else if (tilt == 'R')
	{
		ser->WriteData("R", 1);
		Sleep(200);
		cout << "Strong Right" << "\t\t HoughLine Angle : " << angle << endl;
	}
	else
	{
		cout << "Invalid Input" << endl;
	}
	return 0;
};


int main()
{
	/* 
	* <Serial Communication Settings>
	* This part illustrates
	* defining new serial port and connecting port.
	* For this case, Arduino Uno is connected at port COM6.
	* Change port no. if necessary. 
	*/
	Serial* ser = new Serial("\\\\.\\COM6");
	if (ser->IsConnected()) {
		cout << "Serial Communication Connected" << endl;
	}
	else {
		cout << "Device can not be found or can not be configured" << endl;
	}

	/*
	* <Handle Window Trackbar>
	* OpenCV includes trackbar related functions.
	* Designing trackbars are important because
	* noises occur frequently in computer vision.
	* Blurring the image itself could reduce noises,
	* but several other criteria such as finding color contours,
	* distance from reference point(rho),
	* azimuth angle(theta) from reference
	* and Its threshold must be set to detect object precisely,
	* and draw geometries correctly. 
	* 
	* <Value changes due to trackbar control>
	* rho 1 unit -> 0.2px inc/dec
	* theta 1 unit -> 0.1degree inc/dec
	* pixel vote 1 unit -> 1 vote inc/dec
	*/
	namedWindow("img_color");
	setMouseCallback("img_color", mouse_callback);
	createTrackbar("threshold", "img_color", &threshold1, 255);
	setTrackbarPos("threshold", "img_color", 100);
	namedWindow("Trackbars", WINDOW_NORMAL);
	resizeWindow("Trackbars", Size(600, 180));
	createTrackbar("Rho unit", "Trackbars", &rho_unit, 20);
	createTrackbar("theta unit", "Trackbars", &theta_degree_unit, 50);
	createTrackbar("min vote unit", "Trackbars", &ptr_votes_thres, 100);
	setTrackbarPos("Rho unit", "Trackbars", 5);
	setTrackbarPos("theta unit", "Trackbars", 10);
	setTrackbarPos("min vote unit", "Trackbars", 60);

	/*
	* <Webcam Finding>
	* Default cameras in your device are indexed 0, 
	* but some devices including extra webcam
	* or featuring stereo cameras, depth cameras could be indexed nonzero. (e.g 1,2...etc)
	*/
	VideoCapture cap(0);
	if (!cap.isOpened()) {

		cout << "Cannot open camera." << endl;
		return -1;
	}
	vector<Vec2f> lines;

	/*
	* <Main Loop Of the Code>
	* Part 1 : Image Processing
	* 1-1. Capture Images from Webcam
	* 1-2. Blur Original Images by Gaussian Filter
	* 1-3. Convert Color Coordinates of the Image from BGR(Blue, Green, Red) to HSV(Hue, Saturation, Value)
	* 1-4. Mask Image with Similar Color from the Point that had Once Callbacked
	* 1-5. Remain the Original Image Part of the Image and Eliminate the Leftover Part
	* 1-6. Draw Geometries(Rectanle, Circle) at the Original Image where Masked Objects Detected
	* 1-7. Edge Detection using Canny Edge Algorithm
	* 1-8. Draw HoughLine after Hough Transform, Return Line Information(Startpoint, Endpoint, Angle etc...)
	* 
	* Part 2 : Serial Transmitting
	* 2-1. Get the angle return value from HoughLine
	* 2-2. Define characters to transmit from angle range
	* 2-3. Transmit character to Arduino
	* 
	* Part 3 : Output and Termination
	* 3-1. Visualize Trackbars, Images
	* 3-2. Terminate Code when the user press ESC
	*/
	while (1)
	{
		cap.read(img_color);
		img_color.copyTo(img_color2hough);
		threshold1 = getTrackbarPos("threshold", "img_color");
		Mat img_blur, img_hsv;
		GaussianBlur(img_color2hough, img_blur, Size(3, 3), 0);
		cvtColor(img_blur, img_hsv, COLOR_BGR2HSV);
		
		Mat img_mask1, img_mask2, img_mask3, img_mask;
		inRange(img_hsv, lower_blue1, upper_blue1, img_mask1);
		inRange(img_hsv, lower_blue2, upper_blue2, img_mask2);
		inRange(img_hsv, lower_blue3, upper_blue3, img_mask3);
		img_mask = img_mask1 | img_mask2 | img_mask3;

		int morph_size = 2;
		Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
		morphologyEx(img_mask, img_mask, MORPH_OPEN, element);
		morphologyEx(img_mask, img_mask, MORPH_CLOSE, element);

		Mat img_result;
		bitwise_and(img_color2hough, img_color2hough, img_result, img_mask);

		Mat img_labels, stats, centroids;
		int numofLabels = connectedComponentsWithStats(img_mask, img_labels, stats, centroids,8,CV_32S);
		for (int j = 1; j < numofLabels; j++)
		{
			int area = stats.at<int>(j, CC_STAT_AREA);
			int left = stats.at<int>(j, CC_STAT_LEFT);
			int top = stats.at<int>(j, CC_STAT_TOP);
			int width = stats.at<int>(j, CC_STAT_WIDTH);
			int height = stats.at<int>(j, CC_STAT_HEIGHT);
			int centerX = centroids.at<double>(j, 0);
			int centerY = centroids.at<double>(j, 1);
			if (area > 500)
			{
				circle(img_color, Point(centerX, centerY), 5, Scalar(255, 0, 0), 1);
				rectangle(img_color, Point(left, top), Point(left + width, top + height), Scalar(0, 0, 255), 1);
			}
		}

		Mat img_canny;
		Canny(img_result, img_canny, 50,200);
		double rho_px = double(rho_unit) / 5;
		double theta_radian = (double(theta_degree_unit) / 10) * toRadian;
		HoughLines(img_canny, lines, rho_px, theta_radian, ptr_votes_thres);
		
		double angle = drawHoughLines(lines);
		char tilt;
		if (angle < -30) { tilt = 'L'; }
		else if (angle < -10 && angle >= -30) { tilt = 'l'; }
		else if (angle > 30) { tilt = 'R'; }
		else if (angle > 10 && angle <= 30) { tilt = 'r'; }
		else { tilt = 'N'; }
		int steer = TX_DATA(ser, tilt, angle);

		imshow("img_color", img_color);
		imshow("img_mask", img_mask);
		imshow("img_result", img_result);
		imshow("img_canny", img_canny);
		imshow("img_houghline", img_color2hough);

		if (waitKey(1) == 27)
		{
			cout << "Esc key is pressed by user. Stopping the video" << endl;
			break;
		}
	}
	return 0;
}