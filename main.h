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

// Values for the Threshold/Canny sliders taken from online examples
// These sliders allow visualisation of the thresholding needed for different lighting conditions
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
char* window_name2 = "Threshold Demo";
char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value = "Value";

cv::Mat out2 = cv::Mat(480, 640, CV_8UC4);
cv::Mat depthImage;
cv::Mat checkerImage = cv::Mat(480,640,CV_8U);
USHORT* depthinMM;
cv::Mat transformMat;
cv::Mat src_gray2, dst2;
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
	
};

// Thresholding image with user input slider
void Threshold_Demo(int, void*);

// Run Canny on thresholded image
void CannyThreshold(int, void*);

// Function to connect and initiate stream with the connect
void connectKinect(kinectClass* kinect, bool isColour);

// Function to grab one set of BYTE* data for either colour or depth
// Ensure the stream is open for this type (either colour or depth)
void getKinectData(kinectClass* kinect, bool isColour);

// Run by getKinectData to extract the BYTE* colour data into a cv::Mat
cv::Mat * GetColorImage(kinectClass *kinect, BYTE* bytes, int width, int height);

// Run by getKinectData to extract the (USHORT*) BYTE* depth data into a cv::Mat
// AND map the depth to the colour image pixels and extract this depth for each pixel in an array depthinMM[]
cv::Mat * GetDepthImage(kinectClass *kinect, BYTE* depthData, NUI_LOCKED_RECT* LockedRect, int width, int height);

// Find edges that are the circular edges of cups
std::vector<std::vector<int>> getCupEdges();

// Adjust to the camera coordinate frame (center of camera = (0,0,0))
cv::Point3d adjustCoords(int x, int y, int d);

// Detect the specified chessboard and find the corners
void detectChessboard(cv::Mat * input, cv::Mat * output);

// Find the angle between two vectors
double angleBetweenVectors(cv::Point3d p1, cv::Point3d p2);

// Horns Algorithm to transfer from camera coordinate frame to another
cv::Mat hornsAlgorithm(cv::vector<cv::Point3d> A, cv::vector<cv::Point3d> B);

// Calibrate the kinect and find the fx,fy,cx,cy matrix
void calibrateCamera(kinectClass *kinect);

// Calibrate the kinect from saved images
void calibrateFromImages();


// UNUSED FUNCTIONS: ONLY HERE AS EXAMPLES AND NOT REPRESENTATIVE OF THE PROJECT OR ITS WORKS

// An example surf function for OPENCV
void surfObjects(cv::Mat img_object, cv::Mat img_scene, std::vector<cv::KeyPoint>* keypoints_object, std::vector<cv::KeyPoint>* keypoints_scene, std::vector<cv::DMatch>* good_matches);
// An example sift function for OPENCV
void siftObjects(cv::Mat img_object, cv::Mat img_scene, std::vector<cv::KeyPoint>* keypoints_object, std::vector<cv::KeyPoint>* keypoints_scene, std::vector<cv::DMatch>* good_matches);
