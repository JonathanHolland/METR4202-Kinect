#pragma once
#include <windows.h>
#include <Ole2.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define COLOR_WIDTH 640
#define COLOR_HEIGHT 480
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480

int edgeThresh = 1;
int lowThreshold = 25;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";
cv::Mat dst, detected_edges, src_gray, colour_image;

int threshold_value = 252;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;
cv::Mat out2 = cv::Mat(480, 640, CV_8UC4);
cv::Mat depthImage;
cv::Mat checkerImage = cv::Mat(480,640,CV_8U);
USHORT* depthinMM;
cv::Mat transformMat;
cv::Mat src_gray2, dst2;
double fx = 384.67525007;
double fy = 312.96378221;
double cx = 388.3476765;
double cy = 296.26745933;

cv::Mat cameraMat = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

char* window_name2 = "Threshold Demo";

char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value = "Value";

cv::vector<cv::RotatedRect> cupEllipses;

struct kinectClass
{
	// Kinect variables
	HANDLE depthStream;              // The identifier of the Kinect's RGB Camera
	HANDLE colorStream;				// The identifier of the Kinect's Depth Sensor
	HANDLE nextColorFrameEvent;
	HANDLE nextDepthFrameEvent;
	HRESULT hr;						// Used to identify errors when connecting with the sdk
	INuiSensor* sensor;            // The kinect sensor
	IplImage* img;					// The image gathered from the kinect
	cv::Mat* m;						// The stored mat image after conversion from kinect
	cv::Mat* mcolor;
	BYTE * depthData;
	BYTE * colourData;
	INuiCoordinateMapper* depthToRgbMap;

public:
	// Constructor
	kinectClass() {
	};
	// Destructor
	// Function to initiate connection
	
};

void CannyThreshold(int, void*);
void Threshold_Demo(int, void*);
void connectKinect(kinectClass* kinect, bool isColour);
void getKinectData(kinectClass* kinect, bool isColour);
cv::Mat * GetColorImage(kinectClass *kinect, BYTE* bytes, int width, int height);
cv::Mat * GetDepthImage(kinectClass *kinect, BYTE* depthData, NUI_LOCKED_RECT* LockedRect, int width, int height);
void surfObjects(cv::Mat img_object, cv::Mat img_scene, std::vector<cv::KeyPoint>* keypoints_object, std::vector<cv::KeyPoint>* keypoints_scene, std::vector<cv::DMatch>* good_matches);
void siftObjects(cv::Mat img_object, cv::Mat img_scene, std::vector<cv::KeyPoint>* keypoints_object, std::vector<cv::KeyPoint>* keypoints_scene, std::vector<cv::DMatch>* good_matches);
std::vector<std::vector<int>> getCupEdges();
cv::Point3d adjustCoords(int x, int y, int d);
void detectChessboard(cv::Mat * input, cv::Mat * output);
double angleBetweenVectors(cv::Point3d p1, cv::Point3d p2);
cv::Mat hornsAlgorithm(cv::vector<cv::Point3d> A, cv::vector<cv::Point3d> B);
void calibrateCamera(kinectClass *kinect);
void calibrateFromImages();