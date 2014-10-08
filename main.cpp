#include "main.h"



/*
* The trigger function is bound to OpenCV display windows to display
* specified data on mouse click. depthinMM is passed in via void* userdata 
* but is referenced as a global variable for ease of understanding.
*/
void trigger(int event, int x, int y, int flags, void* userdata)
{
	USHORT* depth = (USHORT*)userdata;
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		printf("Left button of the mouse is clicked - position %d, %d: ", x, y);
		printf("Depth: %d\n", depthinMM[(y+1) * 640 - x]);

		int z10 = (y+1) * 640 - x;
		cv::Mat tempp = cv::Mat(adjustCoords(x, y, depthinMM[z10]));
		//cv::convertPointsToHomogeneous(cv::Mat(adjustCoords(0, 0, 800)), tempp);
		cv::Mat one = (cv::Mat_<double>(1, 1) << 1);
		cv::Mat newvec = (cv::Mat_<double>(4, 1) << tempp.at<double>(0, 0), tempp.at<double>(1, 0), tempp.at<double>(2, 0), one.at<double>(0, 0));
		cv::Mat output = transformMat*newvec;
		printf("Output: %f,%f,%f\n", output.at<double>(0, 0), output.at<double>(1, 0), output.at<double>(2, 0));

	}
}

/*
* The main function 
* - Establishes and passes through the kinectClass object
* - Runs connectKinect and getKinectData for both colour and depth
* - Does the bulk of the calculations on this data
*/
int main()
{
	kinectClass kinect;
	// To calibrate directly from the kinect input
	//calibrateCamera(&kinect);
	// To calibrate from saved images
	calibrateFromImages();

	// Wait 2000ms to allow autofocus to take effect
	// This allows *Some* standardisation of lighting
	cv::waitKey(2000);

	// The kinect won't allow the grabbing of frames if both streams are open at once
	// Open colour stream
	connectKinect(&kinect, true);
	// Grab colour mat and pass to kinect.mcolor
	getKinectData(&kinect, true);
	// Clone colour mat
	cv::Mat colour = kinect.mcolor->clone();
	// Close colour stream handle
	if (kinect.nextColorFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(kinect.nextColorFrameEvent);
	}

	// Open depth stream
	connectKinect(&kinect, false);
	// Grab depth mat and pass to kinect.m
	getKinectData(&kinect, false);
	// Clone depth mat
	cv::Mat depth = kinect.m->clone();

	// Open and view Colour and Depth and assign MouseCallBacks
	// WINDOW_NORMAL is preferable over WINDOW_AUTOSIZE because it will scale
	// to a specified new window size when required
	cv::namedWindow("depth", cv::WINDOW_NORMAL);
	cv::imshow("depth", depth);
	cv::setMouseCallback("depth", trigger, (void*)depthinMM);
	cv::namedWindow("colour", cv::WINDOW_NORMAL);
	cv::imshow("colour", *kinect.mcolor);
	cv::setMouseCallback("colour", trigger, (void*)depthinMM);
	
	// Instantiate variables for using a 8x5 (internal corners) chessboard
	// to se the co-ordinate frame. The same chessboard is used for calibration.

	//detectChessboard(kinect.m, &colour);
	cv::Size patternsize(8, 5); //interior number of corners //source image
	cv::vector<cv::Point2f> corners; //this will be filled by the detected corners
	cv::cvtColor(colour, checkerImage, cv::COLOR_RGBA2GRAY);
	//checkerImage = cv::imread("samples/test/Test2.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat colour2 = colour.clone();
	bool patternfound = findChessboardCorners(checkerImage, patternsize, corners,
		cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
		+ cv::CALIB_CB_FAST_CHECK);

	if (patternfound)
		cornerSubPix(checkerImage, corners, cv::Size(11, 11), cv::Size(-1, -1),
		cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	//cv::drawChessboardCorners(colour2, patternsize, cv::Mat(corners), patternfound);

	cv::vector<cv::Point3f> boardPoints;
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 8; j++) {
			boardPoints.push_back(cv::Point3f(0, i, j));
		}
	}
	cv::vector<double> rvec;
	cv::vector<double> tvec;
	cv::vector<cv::Point3d> points0;
	points0.push_back(cv::Point3d(5, 0, 0));
	points0.push_back(cv::Point3d(0, 5, 0));
	points0.push_back(cv::Point3d(0, 0, 5));
	points0.push_back(cv::Point3d(5, 5, 5));

	cv::vector<cv::Point2d> points2;
	cv::Mat points3 = cv::Mat(3, 1, CV_32FC2);
	cv::vector<double> empty;
	for (int i = 0; i < 5; i++) {
		empty.push_back(0);
	}
	
	// Display values and circle on image the four detected chessboard corners
	cv::vector<cv::Point3d> points5;
	int zdepth = ((int)corners[0].y + 1) * 640 - ((int)corners[0].x);
	points5.push_back(adjustCoords(corners[0].x, corners[0].y, depthinMM[zdepth]));
	printf("Corner location: %f, %f\n", corners[0].x, corners[0].y);
	cv::circle(colour2, corners[0], 10, cv::Scalar(0, 255, 0));

	printf("Zdepth: %d\n", depthinMM[zdepth]);
	int zdepth2 = ((int)corners[7].y + 1) * 640 - ((int)corners[7].x);
	points5.push_back(adjustCoords(corners[7].x, corners[7].y, depthinMM[zdepth2]));
	printf("Corner location: %f, %f\n", corners[7].x, corners[7].y);
	cv::circle(colour2, corners[7], 10, cv::Scalar(0, 255, 0));

	printf("Zdepth: %d\n", depthinMM[zdepth2]);
	int zdepth3 = ((int)corners[32].y+1) * 640 - ((int)corners[32].x);
	points5.push_back(adjustCoords(corners[32].x, corners[32].y, depthinMM[zdepth3]));
	printf("Corner location: %f, %f\n", corners[32].x, corners[32].y);
	cv::circle(colour2, corners[32], 10, cv::Scalar(0, 255, 0));

	printf("Zdepth: %d\n", depthinMM[zdepth3]);
	int zdepth4 = ((int)corners[39].y+1) * 640 - ((int)corners[39].x);
	points5.push_back(adjustCoords(corners[39].x, corners[39].y, depthinMM[zdepth4]));
	printf("Corner location: %f, %f\n", corners[39].x, corners[39].y);
	cv::circle(colour2, corners[39], 10, cv::Scalar(0, 255, 0));

	printf("Zdepth: %d\n", depthinMM[zdepth4]);
	cv::vector<cv::Point3d> points6;
	points6.push_back(cv::Point3d(0, 55, 60));// 0 54 59
	points6.push_back(cv::Point3d(0, 55, 241));// 0 54 241
	points6.push_back(cv::Point3d(0, 158, 61));// 0 158 59
	points6.push_back(cv::Point3d(0, 158, 242));// 0 158 241

	// Use Horns Algorithm to use these points to transform the camera coordinate frame
	// to the world coordinate frame (at the corner of the chessboard)
	transformMat = hornsAlgorithm(points5, points6);
	printf("transformMat: %f, %f, %f, %f\n", transformMat.at<double>(0, 0), transformMat.at<double>(0, 1), transformMat.at<double>(0, 2), transformMat.at<double>(0, 3));
	printf("transformMat: %f, %f, %f, %f\n", transformMat.at<double>(1, 0), transformMat.at<double>(1, 1), transformMat.at<double>(1, 2), transformMat.at<double>(1, 3));
	printf("transformMat: %f, %f, %f, %f\n", transformMat.at<double>(2, 0), transformMat.at<double>(2, 1), transformMat.at<double>(2, 2), transformMat.at<double>(2, 3));

	cv::waitKey(0);
	

	cv::Mat img_object;
	cv::cvtColor(colour, img_object, CV_RGBA2GRAY);
	src_gray = img_object.clone();
	src_gray2 = img_object.clone();
	if (!img_object.data)
	{
		std::cout << " --(!) Error reading image " << std::endl; return -2;
	}

	cv::namedWindow(window_name2, CV_WINDOW_NORMAL);

	/// Create Trackbar to choose type of Threshold
	cv::createTrackbar(trackbar_type,
		window_name2, &threshold_type,
		max_type, Threshold_Demo);

	cv::createTrackbar(trackbar_value,
		window_name2, &threshold_value,
		max_value, Threshold_Demo);

	// Call the function to initialize
	Threshold_Demo(0, 0);

	cv::waitKey(0);
	
	dst.create(img_object.size(), img_object.type());
	cv::namedWindow(window_name, CV_WINDOW_NORMAL);
	cv::createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);
	CannyThreshold(0, 0);

	cv::waitKey(0);
	
	

	// Release handles we created
	// Comment out Color and Uncomment depth if using depth frame first
	// Then do the opposite in connectKinect
	// IF running the above in a loop for subsequent images from the kinect feed
	// make sure to only close the handles AFTER the stream is no longer used.
	if (kinect.nextColorFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(kinect.nextColorFrameEvent);
	}
	//if (kinect.nextDepthFrameEvent != INVALID_HANDLE_VALUE)
	//{
	//	CloseHandle(kinect.nextDepthFrameEvent);
	//}

	// Shutdown kinect
	kinect.sensor->NuiShutdown();
	kinect.sensor->Release();

	return 0;
}

void Threshold_Demo(int, void*)
{
	/* 0: Binary
	1: Binary Inverted
	2: Threshold Truncated
	3: Threshold to Zero
	4: Threshold to Zero Inverted
	*/
	threshold(src_gray2, dst2, threshold_value, max_BINARY_value, threshold_type);

	imshow(window_name2, dst2);
}

void CannyThreshold(int, void*)
{
	cupEllipses.clear();
	/// Reduce noise with a kernel 3x3
	cv::vector < cv::vector < cv::Point>> contours;
	cv::vector <cv::Vec4i> hierarchy;
	blur(dst2, detected_edges, cv::Size(3, 3));
	cv::Mat m3 = src_gray.clone();
	cv::cvtColor(m3, m3, CV_GRAY2RGB);

	/// Canny detector
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
	cv::findContours(detected_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	cv::vector<cv::RotatedRect> minEllipse(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		if (contours[i].size() > 30) {
			cv::RotatedRect r = cv::fitEllipse(cv::Mat(contours[i]));
			if ((r.boundingRect().height < r.boundingRect().width)) {
				bool flag = true;
				for (int j = 0; j < cupEllipses.size(); j++) {
					if ((cupEllipses[j].boundingRect().contains(r.center) && cupEllipses[j].boundingRect().area() > r.boundingRect().area())) {
						flag = false;
					}
				}
				if (flag) {
					minEllipse[i] = r;
					cupEllipses.push_back(r);
					printf("%d,%d \n", r.boundingRect().width, r.boundingRect().height);
					cv::ellipse(m3, minEllipse[i], cv::Scalar(255, 0, 0), 2, 8);
				}
			}
		}
	}
	//cv::Mat drawing = cv::Mat::zeros(detected_edges.size(),CV_8UC3);
	for (int i = 0; i < contours.size(); i++) {
		if (minEllipse[i].center.x == 0) {
			continue;
		}
		cv::Scalar color = cv::Scalar(255,0,0);
	}
	std::vector<std::vector<int>> edges = getCupEdges();
	for (int i = 0; i < edges.size(); i++) {
		cv::line(m3, cv::Point2f(edges[i][0], edges[i][1]), cv::Point2f(edges[i][2], edges[i][1]), cv::Scalar(0, 255, 0), 4);
		cv::line(m3, cv::Point2f(edges[i][2], edges[i][1]), cv::Point2f(edges[i][2], edges[i][3]), cv::Scalar(0, 255, 0), 4);
		cv::line(m3, cv::Point2f(edges[i][2], edges[i][3]), cv::Point2f(edges[i][0], edges[i][3]), cv::Scalar(0, 255, 0), 4);
		cv::line(m3, cv::Point2f(edges[i][0], edges[i][3]), cv::Point2f(edges[i][0], edges[i][1]), cv::Scalar(0, 255, 0), 4);
	}
	cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	cv::imshow("Contours", m3);

	/// Using Canny's output as a mask, we display our result
	dst = cv::Scalar::all(0);

	src_gray.copyTo(dst, detected_edges);
	imshow(window_name, dst);
}

// get a CV_8UC4 (RGB) Matrix from Kinect RGB frame
cv::Mat * GetColorImage(kinectClass *kinect, BYTE* bytes, int width, int height)
{
	const unsigned int img_size = width*height*4;
	cv::Mat * out = new cv::Mat(height, width, CV_8UC4);
	memcpy(out->data, bytes, img_size);
	cv::flip(*out, out2, 1);
	return &out2;
}

// get a CV_8U matrix from a Kinect depth frame 
cv::Mat * GetDepthImage(kinectClass *kinect, BYTE* depthData, NUI_LOCKED_RECT* LockedRect, int width, int height)
{
	
	// We run the below to map the depth image to the color image
	USHORT* data = (USHORT*)depthData;
	IplImage* out = cvCreateImage(cvSize(640, 480), 8, 1);
	int r = 480;
	int c = 640;
	long *colorCoords = new long[r*c * 2];

	kinect->sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		NUI_IMAGE_RESOLUTION_640x480,
		NUI_IMAGE_RESOLUTION_640x480,
		r*c,
		data,
		r*c*2,
		colorCoords
		);
	
	depthinMM = new USHORT[LockedRect->size];
	for (int i = 0; i < LockedRect->size; i += sizeof(USHORT)) {
		unsigned char* pBuf = (unsigned char*)LockedRect->pBits + i;
		unsigned short* pBufS = (unsigned short*)pBuf;
		unsigned short depth = ((*pBufS &0xfff8)>> 3);
		// The following function achieves the same result as the above & and shift
		//unsigned short depth = NuiDepthPixelToDepth(*pBufS);
		unsigned char intensity = depth > 0 ? 255 - (unsigned char)(256 * depth / 0x0fff) : 0;

		long
			x = colorCoords[i],
			y = colorCoords[i + 1];

		
		if (x >= 0 && x < out->width && y >= 0 && y < out->height) {
			out->imageData[x + y * out->widthStep] = intensity;
			depthinMM[x + y*out->widthStep] = depth;
		}

	}

	
	// Convert IplImage to cv::Mat
	depthImage = cv::cvarrToMat(out, false);
	
	return &depthImage;
}

// Connect to the kinect and open either the depth or color stream
void connectKinect(kinectClass * kinect, bool flag) {
	//NUI initialize the elements we need for the kinect
	DWORD nuiFlags = NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_DEPTH;
	kinect->hr = NuiCreateSensorByIndex(0, &kinect->sensor);
	kinect->hr = NuiInitialize(nuiFlags);
	if (FAILED(kinect->hr))
	{
		printf("nui_initialize failed \n");
		return;
	}
	else printf("nui initialized ! \n");

	if (flag == true) {

		//create the colour image stream and connect
		kinect->colorStream = INVALID_HANDLE_VALUE;
		kinect->nextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		kinect->hr = NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,			// COLOR or DEPTH?
			NUI_IMAGE_RESOLUTION_640x480, // Resolution
			0,						// Image stream flags, e.g. near mode
			2,					  // The number of frames to buffer
			kinect->nextColorFrameEvent, // The next instance event handle
			&kinect->colorStream); // The reference handle for this stream
		if (FAILED(kinect->hr))
		{
			printf("colour video stream failed to open ! \n");
		}
		else printf("colour video stream opened ! \n");

	}
	else 
	{
		//create the depth image stream and connect
		kinect->depthStream = INVALID_HANDLE_VALUE;
		kinect->nextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		kinect->hr = NuiImageStreamOpen(
			NUI_IMAGE_TYPE_DEPTH,			// COLOR or DEPTH?
			NUI_IMAGE_RESOLUTION_640x480, // Resolution
			0,						// Image stream flags, e.g. near mode
			2,					  // The number of frames to buffer
			kinect->nextDepthFrameEvent, // The next instance event handle
			&kinect->depthStream); // The reference handle for this stream
		if (FAILED(kinect->hr))
		{
			printf("depth video stream failed to open ! \n");
		}
		else printf("depth video stream opened ! \n");	
	
	}
	
	return;
}

// Grab data for whichever stream is open
void getKinectData(kinectClass * kinect, bool isColour) {
	INuiFrameTexture * pTexture;
	const NUI_IMAGE_FRAME* imageFrame;
	NUI_IMAGE_FRAME imageFrame2;
	if (isColour) {
		// wait for event to be triggered by Kinect (wait forever)
		WaitForSingleObject(kinect->nextColorFrameEvent, INFINITE);

		kinect->hr = NuiImageStreamGetNextFrame(kinect->colorStream, 0, &imageFrame);
		if (FAILED(kinect->hr))
		{
			printf("get frame failed \n");
		}
		else printf("get frame \n");

		pTexture = imageFrame->pFrameTexture;
		NUI_LOCKED_RECT LockedRect;
		// Lock the frame data so the Kinect knows not to modify it while we're reading it
		pTexture->LockRect(0, &LockedRect, NULL, 0);
		if (LockedRect.Pitch != 0)
		{
			kinect->colourData = LockedRect.pBits;

			// Colour specific format
			kinect->mcolor = GetColorImage(kinect,kinect->colourData, COLOR_WIDTH, COLOR_HEIGHT);
		}
		else
		{
			printf("Received colour textures pitch is zero \n");
		}
		// We're done with the texture so unlock it
		pTexture->UnlockRect(0);

		// Release the frame
		NuiImageStreamReleaseFrame(kinect->colorStream, imageFrame);
	}
	else {
		// wait for event to be triggered by Kinect (wait forever)
		WaitForSingleObject(kinect->nextDepthFrameEvent, INFINITE);

		
		BOOL nearMode;
		kinect->hr = NuiImageStreamGetNextFrame(kinect->depthStream, 0, &imageFrame);
		//kinect->hr = kinect->sensor->NuiImageFrameGetDepthImagePixelFrameTexture(kinect->depthStream, &imageFrame2, &nearMode, &pTexture);

		if (FAILED(kinect->hr))
		{
			printf("get frame failed \n");
		}
		else printf("get frame \n");

		pTexture = imageFrame->pFrameTexture;
		NUI_LOCKED_RECT LockedRect;
		// Lock the frame data so the Kinect knows not to modify it while we're reading it
		pTexture->LockRect(0, &LockedRect, NULL, 0);
		if (LockedRect.Pitch != 0)
		{
			kinect->depthData = LockedRect.pBits;
			// Depth specific format
			kinect->m = GetDepthImage(kinect,kinect->depthData,&LockedRect, DEPTH_WIDTH, DEPTH_HEIGHT);
		}
		else
		{
			printf("Buffer length of received texture is bogus \n");
		}
		// We're done with the texture so unlock it
		pTexture->UnlockRect(0);

		// Release the frame
		NuiImageStreamReleaseFrame(kinect->depthStream, imageFrame);
	}
	
}

std::vector<std::vector<int>> getCupEdges() {
	std::vector<std::vector<int>> output;
	std::vector<int> cupSizes;
	for (int i = 0; i < cupEllipses.size(); i++) {
		std::vector<int> cupOutput;
		// get the depth of the ellipse
		float x = cupEllipses.at(i).center.x;
		if (x < 0) {
			x = 0;
		}
		float ybegin = 0;
		float yend = 0;
		float yend2 = 0;
		int leftOfCup = 0;
		int rightOfCup = 0;
		if (cupEllipses.at(i).size.height < cupEllipses.at(i).size.width) {
			ybegin = cupEllipses.at(i).center.y - (cupEllipses.at(i).size.height / 2);
			yend = ybegin + cupEllipses.at(i).size.height;
			yend2 = yend + 3 * cupEllipses.at(i).size.width*(1 - cupEllipses.at(i).size.height / cupEllipses.at(i).size.width);
			leftOfCup = cupEllipses.at(i).center.x - cupEllipses.at(i).size.width / 2;
			rightOfCup = leftOfCup + cupEllipses.at(i).size.width;

			
		}
		else {
			ybegin = cupEllipses.at(i).center.y - (cupEllipses.at(i).size.width / 2);
			yend = ybegin + cupEllipses.at(i).size.width;
			yend2 = yend + 3 * cupEllipses.at(i).size.height*(1 - cupEllipses.at(i).size.width / cupEllipses.at(i).size.height);
			leftOfCup = cupEllipses.at(i).center.x - cupEllipses.at(i).size.height / 2;
			rightOfCup = leftOfCup + cupEllipses.at(i).size.height;

		}

		if (ybegin < 0) {
			ybegin = 0;
		}
		printf("%f, %f ", ybegin, yend);
		cv::vector<float> ellipseDepths;
		for (int y = ybegin; y < yend; y++) {
			int z = y * 640 + x;
			if (depthinMM[z] < 5000) {
				ellipseDepths.push_back(depthinMM[z]);
			}
			printf("%u ", depthinMM[z]);
		}
		cv::vector<float> cupDepths;

		for (int y = yend + 10; y < yend2; y++) {
			int z = y * 640 + x;
			if (depthinMM[z] < 5000) {
				cupDepths.push_back(depthinMM[z]);
			}
			//printf("%u ", depthinMM[z]);

		}
		auto result = std::minmax_element(cupDepths.begin(), cupDepths.end());
		int bottomOfCup = (result.second - cupDepths.begin()) + yend + 10;

		int z1 = cupEllipses.at(i).center.y * 640 + cupEllipses.at(i).center.x;
		int z2 = bottomOfCup * 640 + cupEllipses.at(i).center.x;
		if (depthinMM[z1] == 0) {
			cupSizes.push_back(cupDepths[0] * (adjustCoords(0, bottomOfCup, depthinMM[z2 + 1]).y - adjustCoords(0, yend, depthinMM[z2 + 1]).y));
		} else {
			cupSizes.push_back(cupDepths[0] * (adjustCoords(0, bottomOfCup, depthinMM[z2]).y - adjustCoords(0, yend, depthinMM[z2]).y));
		}
		

		printf("\n");

		cupOutput.push_back(leftOfCup);
		cupOutput.push_back(ybegin);
		cupOutput.push_back(rightOfCup);
		cupOutput.push_back(bottomOfCup);
		output.push_back(cupOutput);
	}
	
	for (int i = 0; i < cupSizes.size(); i++) {
		printf("Cupsize: %d \n", cupSizes[i]);
	}

	return output;
}

cv::Point3d adjustCoords(int x, int y, int d) {
	// The values of the camera calibration matrix found from calibration
	double fx = 384.67525007;
	double fy = 312.96378221;
	double cx = 388.3476765;
	double cy = 296.26745933;

	// The adjustment without taking into account the distortion coefficients
	double newx = (x - cx) * d / fx;
	double newy = (y - cy) * d / fy;

	return cv::Point3d(newx, newy, d);
}

void detectChessboard(cv::Mat * input, cv::Mat * output) {
	cv::Size patternsize(8, 5); //interior number of corners //source image
	cv::vector<cv::Point2f> corners; //this will be filled by the detected corners

	// Find the corners
	bool patternfound = findChessboardCorners(*input, patternsize, corners,
		cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
		+ cv::CALIB_CB_FAST_CHECK);

	if (patternfound)
		cornerSubPix(*input, corners, cv::Size(11, 11), cv::Size(-1, -1),
		cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	// Draw the corners
	cv::drawChessboardCorners(checkerImage, patternsize, cv::Mat(corners), patternfound);
}

double angleBetweenVectors(cv::Point3d p1, cv::Point3d p2) {
	double dot = p1.ddot(p2);
	return acos(dot / (sqrt(p1.ddot(p1))*sqrt(p2.ddot(p2))));
}

cv::Mat hornsAlgorithm(cv::vector<cv::Point3d> A, cv::vector<cv::Point3d> B) {
	int numPoints = A.size();
	//double Asum = 0;
	//double Bsum = 0;
	cv::Point3d Asum = cv::Point3d(0,0,0);
	cv::Point3d Bsum = cv::Point3d(0,0,0);
	for (int i = 0; i < A.size(); i++) {
		Asum = Asum + A[i];
		Bsum = Bsum + B[i];
	}
	
	cv::Point3d Ca = cv::Point3d((Asum.x / numPoints), (Asum.y / numPoints), (Asum.z / numPoints));
	cv::Point3d Cb = cv::Point3d((Bsum.x /numPoints), (Bsum.y / numPoints), (Bsum.z / numPoints));
	cv::vector<cv::Point3d> An;
	cv::vector<cv::Point3d> Bn;
	for (int i = 0; i < A.size(); i++) {
		An.push_back(A[i] - Ca);
		Bn.push_back(B[i] - Cb);
	}
	cv::Mat M = (cv::Mat_<double>(4,4) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	for (int i = 0; i < numPoints; i++) {
		double a1 = 0;
		double a2 = An[i].x;
		double a3 = An[i].y;
		double a4 = An[i].z;
		double b1 = 0;
		double b2 = Bn[i].x;
		double b3 = Bn[i].y;
		double b4 = Bn[i].z;
		cv::Mat Ma = (cv::Mat_<double>(4, 4) << a1, -a2, -a3, -a4, a2, a1, a4, -a3, a3, -a4, a1, a2, a4, a3, -a2, a1);
		cv::Mat Mb = (cv::Mat_<double>(4, 4) << b1, -b2, -b3, -b4, b2, b1, -b4, b3, b3, b4, b1, -b2, b4, -b3, b2, b1);
		cv::Mat Matr = Ma.t();
		M = M + Matr*Mb;
	}
	cv::Mat eigenVals;
	
	cv::Mat eigenVects;
	cv::eigen(M, true, eigenVals,eigenVects);
	//printf("Eigenvals: %f, %f, %f, %f\n", eigenVals.at<double>(0, 0), eigenVals.at<double>(1, 0), eigenVals.at<double>(2, 0), eigenVals.at<double>(3, 0));
	double e1 = eigenVects.at<double>(0, 0);
	double e2 = eigenVects.at<double>(0, 1);
	double e3 = eigenVects.at<double>(0, 2);
	double e4 = eigenVects.at<double>(0, 3);
	cv::Mat M1 = (cv::Mat_<double>(4, 4) << e1, -e2, -e3, -e4, e2, e1, e4, -e3, e3, -e4, e1, e2, e4, e3, -e2, e1);
	cv::Mat M2 = (cv::Mat_<double>(4, 4) << e1, -e2, -e3, -e4, e2, e1, -e4, e3, e3, e4, e1, -e2, e4, -e3, e2, e1);
	cv::Mat R = M1.t()*M2;
	cv::Mat R1 = R.colRange(1, 4).rowRange(1, 4).clone();

	int s = 1;

	cv::Mat T = cv::Mat(Cb) - s*R1*cv::Mat(Ca);

	cv::Mat result;
	cv::hconcat(R1, T, result);
	return result;
}

// The function to calibrate the kinect camera using a number of images with the checkerboard
// in different postions (both rotated and translated different amounts)
// >10 sample images is recommended
// This function also writes the images from the kinect
// so that calibration can occur while disconnected
void calibrateCamera(kinectClass *kinect) {
	int l = 0;
	connectKinect(kinect, true);
	cv::vector<cv::vector<cv::Point2d>> vectorOfVectors;
	cv::vector<cv::vector<cv::Point2d>> boardPoints;
	cv::vector<cv::Point2d> checkerBoard;
	for (int i = 0; i < 5; i++){
		for (int j = 0; j < 8; j++) {
			checkerBoard.push_back(cv::Point2d(i * 26, j * 26));
		}
	}
	// 14 sample images are used
	// examples stored in images/calibration
	while (l < 15) {
		getKinectData(kinect, true);
		std::ostringstream string;
		string << "Calib" << l<<".jpg";
		cv::imwrite(string.str(),*kinect->mcolor);
		cv::namedWindow("Calibration", CV_WINDOW_AUTOSIZE);
		cv::imshow("Calibration", *kinect->mcolor);
		cv::vector<cv::Point2d> corners;
		cv::findChessboardCorners(*kinect->mcolor,cv::Size(8,5),corners);
		vectorOfVectors.push_back(corners);
		boardPoints.push_back(checkerBoard);
		l++;
		cv::waitKey(500);
	}
	cv::Mat cameraOutputMatrix;
	cv::Mat distCoef;
	cv::vector<cv::vector<double>> rvecs;
	cv::vector<cv::vector<double>> tvecs;
	cv::calibrateCamera(boardPoints, vectorOfVectors, cv::Size(640, 480), cameraOutputMatrix, distCoef, rvecs, tvecs);
	printf("R: %f, %f, %f\n", cameraOutputMatrix.at<double>(0, 0), cameraOutputMatrix.at<double>(0, 1), cameraOutputMatrix.at<double>(0, 2));
	printf("R: %f, %f, %f\n", cameraOutputMatrix.at<double>(1, 0), cameraOutputMatrix.at<double>(1, 1), cameraOutputMatrix.at<double>(1, 2));
	printf("R: %f, %f, %f\n", cameraOutputMatrix.at<double>(2, 0), cameraOutputMatrix.at<double>(2, 1), cameraOutputMatrix.at<double>(2, 2));
}

// This function is similar to calibrateCamera() but instead reads in images
// and does not imwrite() any
void calibrateFromImages() {
	int l = 0;
	cv::vector<cv::vector<cv::Point2d>> vectorOfVectors;
	cv::vector<cv::vector<cv::Point2d>> boardPoints;
	cv::vector<cv::Point2d> checkerBoard;
	for (int i = 0; i < 5; i++){
		for (int j = 0; j < 8; j++) {
			checkerBoard.push_back(cv::Point2d(i * 26, j * 26));
		}
	}
	while (l < 15) {
		std::ostringstream string;
		string << "Calib" << l << ".jpg";
		cv::Mat img = cv::imread(string.str());
		cv::vector<cv::Point2d> corners;
		bool flag = cv::findChessboardCorners(img, cv::Size(8, 5), corners);
		if (flag) {
			vectorOfVectors.push_back(corners);
			boardPoints.push_back(checkerBoard);
		}
		
		l++;
	}
	cv::Mat cameraOutputMatrix; //= cv::Mat::eye(3,3,CV_64F);
	cv::Mat distCoef; //= cv::Mat::zeros(8, 1, CV_64F);
	cv::vector<cv::Mat> rvecs;
	cv::vector<cv::Mat> tvecs;
	cv::calibrateCamera(boardPoints, vectorOfVectors, cv::Size(640, 480), cameraOutputMatrix, distCoef, rvecs, tvecs);
	printf("R: %f, %f, %f\n", cameraOutputMatrix.at<double>(0, 0), cameraOutputMatrix.at<double>(0, 1), cameraOutputMatrix.at<double>(0, 2));
	printf("R: %f, %f, %f\n", cameraOutputMatrix.at<double>(1, 0), cameraOutputMatrix.at<double>(1, 1), cameraOutputMatrix.at<double>(1, 2));
	printf("R: %f, %f, %f\n", cameraOutputMatrix.at<double>(2, 0), cameraOutputMatrix.at<double>(2, 1), cameraOutputMatrix.at<double>(2, 2));
}