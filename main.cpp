#include "main.h"
#include <math.h>

int main()
{
	kinectClass kinect;
	//connectKinect(&kinect, true);
	// The kinect won't allow the grabbing of frames if both streams are open at once
	connectKinect(&kinect, false);
	getKinectData(&kinect, false);
	cv::namedWindow("colourDepth", cv::WINDOW_AUTOSIZE);
	
	cv::imshow("colourDepth", *kinect.m);
	cv::Mat depth = kinect.m->clone();

	if (kinect.nextDepthFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(kinect.nextDepthFrameEvent);
	}
	connectKinect(&kinect, true);
	cv::waitKey(1000);
	getKinectData(&kinect, true);
	cv::Mat colour = kinect.m->clone();
	//detectChessboard(kinect.m, &colour);
	cv::Size patternsize(8, 5); //interior number of corners //source image
	cv::vector<cv::Point2f> corners; //this will be filled by the detected corners

	//CALIB_CB_FAST_CHECK saves a lot of time on images
	//that do not contain any chessboard corners
	cv::cvtColor(colour, checkerImage, cv::COLOR_RGBA2GRAY);
	cv::Mat colour2 = colour.clone();
	bool patternfound = findChessboardCorners(checkerImage, patternsize, corners,
		cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
		+ cv::CALIB_CB_FAST_CHECK);

	if (patternfound)
		cornerSubPix(checkerImage, corners, cv::Size(11, 11), cv::Size(-1, -1),
		cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	
	cv::drawChessboardCorners(colour2, patternsize, cv::Mat(corners), patternfound);
	
	cv::vector<cv::Point3f> boardPoints;
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 8; j++) {
			boardPoints.push_back(cv::Point3f(j , i , 0));
		}
	}
	cv::vector<double> rvec;
	cv::vector<double> tvec;
	cv::vector<cv::Point3d> points0;
	points0.push_back(cv::Point3d(5, 0, 0));
	points0.push_back(cv::Point3d(0, 5, 0));
	points0.push_back(cv::Point3d(0, 0, 5));

	cv::vector<cv::Point2d> points2;
	cv::Mat points3 = cv::Mat(3, 1, CV_32FC2);
	cv::vector<double> empty;
	for (int i = 0; i < 5; i++) {
		empty.push_back(0);
	}

	cv::solvePnP(cv::Mat(boardPoints), corners, cameraMat, empty, rvec, tvec);
	printf("rvec: %f,%f,%f", rvec[0], rvec[1], rvec[2]);
	printf("tvec: %f,%f,%f", tvec[0], tvec[1], tvec[2]);
	cv::projectPoints(points0, rvec, tvec, cameraMat, empty, points2);
	//cv::line(colour2, cv::Point2f(0,0), cv::Point2f(points3.at<cv::Vec2f>(0,0)), cv::Scalar(0, 255, 0), 4);
	//cv::line(colour2, cv::Point2f(0, 0), cv::Point2f(points3.at<cv::Vec2f>(1,0)), cv::Scalar(255, 0, 0), 4);
	//cv::line(colour2, cv::Point2f(0, 0), cv::Point2f(points3.at<cv::Vec2f>(2,0)), cv::Scalar(0, 0, 255), 4);
	cv::line(colour2, corners[0], points2[0], cv::Scalar(0, 255, 0), 4);
	cv::line(colour2, corners[0], points2[1], cv::Scalar(255, 0, 0), 4);
	cv::line(colour2, corners[0], points2[2], cv::Scalar(0, 0, 255), 4);


	cv::namedWindow("colour2", cv::WINDOW_AUTOSIZE);
	cv::imshow("colour2", colour2);
	/*/// Reduce noise with a kernel 3x3
	cv::vector < cv::vector < cv::Point>> contours;
	cv::vector <cv::Vec4i> hierarchy;
	cv::imshow("Image",*kinect.m);
	blur(*kinect.m, detected_edges, cv::Size(3, 3));

	/// Canny detector
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
	cv::findContours(detected_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	
	
	cv::resize(depth, depth, cv::Size(640, 480));
	cv::Mat matrix = cv::Mat(2, 3, CV_32FC1);
	matrix.at<float>(0, 0) = 0.98;
	matrix.at<float>(0, 1) = 0;
	matrix.at<float>(0, 2) = -13;
	matrix.at<float>(1, 0) = 0;
	matrix.at<float>(1, 1) = 1;
	matrix.at<float>(1, 2) = 7;
	cv::warpAffine(depth, depth, matrix,depth.size());

	for (int i = 0; i < contours.size(); i++) {
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(depth, contours, i, color, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point());
	}

	cv::namedWindow("depthoncolour", CV_WINDOW_NORMAL);
	cv::imshow("depthoncolour", depth);
*/



	//cv::Mat img_object = cv::imread("samples/test/Test6.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat img_object;
	cv::cvtColor(colour, img_object, CV_RGBA2GRAY);
	//cv::Mat img_object = *kinect.m;
	cv::Mat img_scene  = cv::imread("samples/test/Test3.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	src_gray = img_object.clone();
	src_gray2 = img_object.clone();
	if (!img_object.data || !img_scene.data)
	{
		std::cout << " --(!) Error reading images " << std::endl; return -2;
	}


	//// The code below shows that for the Threshold to Zero method, a value of 142 leaves the cups with the largest pools of black
	//// Can we use this information to help identify them?
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
	
	// The below transformation using canny tells us that an approx threshold of 50 gives us the cup outline
	dst.create(img_object.size(), img_object.type());
	cv::namedWindow(window_name, CV_WINDOW_NORMAL);
	cv::createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);
	CannyThreshold(0, 0);
	
	// after canny threshold we can look at the array of cupEllipses
	


	cv::waitKey(0);
	
	
	
	int i = 1;
	std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
	std::vector<cv::DMatch> good_matches;
	//threshold(img_scene, img_scene, threshold_value, max_BINARY_value, threshold_type);
	//threshold(img_object, img_object, threshold_value, max_BINARY_value, threshold_type);
	siftObjects(img_object, img_scene, &keypoints_object, &keypoints_scene, &good_matches);

	std::string var = "medium";

	std::vector<cv::Point2f> obj;
	std::vector<cv::Point2f> scene;

	
	//for each image in collection
	//while (i < 7) {
	//	cv::Mat img_object = cv::imread("samples/train/"+var+"/"+std::to_string(i)+".jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//	//cv::namedWindow("bla", 1);
	//	//cv::imshow("bla", img_object);
	//	siftObjects(img_object, img_scene, &keypoints_object, &keypoints_scene, &good_matches);
	//	
	//	i++;
	//}
	
	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
	}

	//std::vector<std::vector<cv::DMatch>> groupedMatches;
	//for (int i = 0; i < good_matches.size; i++) {
	//	for (int j = 0; j < groupedMatches.size; j++) {
	//		
	//		cv::Point2f object = keypoints_object[good_matches[i].trainIdx].pt;
	//		cv::Point2f scene = keypoints_scene[good_matches[i].queryIdx].pt;
	//		double gradient = atan2(scene.y - object.y, scene.x - object.x);
	//		if (gradient -)
	//	}

	//}

	// drawing the results
	cv::namedWindow("matches", 1);
	cv::Mat img_matches;
	cv::drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches);
	imshow("matches", img_matches);

	//cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC);
	
	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0); obj_corners[1] = cvPoint(img_object.cols, 0);
	obj_corners[2] = cvPoint(img_object.cols, img_object.rows); obj_corners[3] = cvPoint(0, img_object.rows);
	std::vector<cv::Point2f> scene_corners(4);

	//cv::perspectiveTransform(obj_corners, scene_corners, H);

	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	//cv::line(img_matches, scene_corners[0] + cv::Point2f(img_object.cols, 0), scene_corners[1] + cv::Point2f(img_object.cols, 0), cv::Scalar(0, 255, 0), 4);
	//cv::line(img_matches, scene_corners[1] + cv::Point2f(img_object.cols, 0), scene_corners[2] + cv::Point2f(img_object.cols, 0), cv::Scalar(0, 255, 0), 4);
	//cv::line(img_matches, scene_corners[2] + cv::Point2f(img_object.cols, 0), scene_corners[3] + cv::Point2f(img_object.cols, 0), cv::Scalar(0, 255, 0), 4);
	//cv::line(img_matches, scene_corners[3] + cv::Point2f(img_object.cols, 0), scene_corners[0] + cv::Point2f(img_object.cols, 0), cv::Scalar(0, 255, 0), 4);

	//cv::line(img_scene, scene_corners[0], scene_corners[1], cv::Scalar(0, 255, 0), 4);
	//cv::line(img_scene, scene_corners[1], scene_corners[2], cv::Scalar(0, 255, 0), 4);
	//cv::line(img_scene, scene_corners[2], scene_corners[3], cv::Scalar(0, 255, 0), 4);
	//cv::line(img_scene, scene_corners[3], scene_corners[0], cv::Scalar(0, 255, 0), 4);

	// IF the elements of scene_corners make a rectangle
	// Keep the drawn lines
	// Otherwise scrap it for this image and keep iterating through the training images

	//-- Show detected matches
	//imshow("Good Matches & Object detection", img_matches);
	imshow("Good Matches & Object detection", img_scene);

	cvWaitKey(0);

	// release handles we created
	if (kinect.nextColorFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(kinect.nextColorFrameEvent);
	}
	//if (kinect.nextDepthFrameEvent != INVALID_HANDLE_VALUE)
	//{
	//	CloseHandle(kinect.nextDepthFrameEvent);
	//}

	//shutdown kinect
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
	//threshold_value = 142;
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
		//cv::drawContours(m3, contours, i, color, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point());
		cv::threshold(src_gray, dst2, 142, max_BINARY_value, 3);
		float yCupSize;
		std::string cupType;
		yCupSize = 50;
		/*if ((dst2.at<uchar>(minEllipse[i].center.y + 40, minEllipse[i].center.x) == 0) && (dst2.at<uchar>(minEllipse[i].center.y + 70, minEllipse[i].center.x) != 0)) {
			//cupType. = 'small';
			yCupSize = 50;
		}
		else if ((dst2.at<uchar>(minEllipse[i].center.y + 70, minEllipse[i].center.x) == 0) && (dst2.at<uchar>(minEllipse[i].center.y + 100, minEllipse[i].center.x) != 0)) {
			//cupType = 'medium';
			yCupSize = 80;
		}
		else {
			//cupType = 'large';
			yCupSize = 100;
		}*/
		/*if ((minEllipse[i].size.area() < (float)9000.0)&&(minEllipse[i].size.area() > (float)2000.0)&&
			(minEllipse[i].size.width <50)&&(minEllipse[i].size.height <200)) {
			cv::ellipse(m3, minEllipse[i], color, 2, 8);

			/*if (dst2.at<uchar>(minEllipse[i].center.y + yCupSize, minEllipse[i].center.x) == 0) {
				cv::rectangle(m3, cv::Point(minEllipse[i].center.x - yCupSize / 1.5, minEllipse[i].center.y),
					cv::Point(minEllipse[i].center.x + yCupSize / 1.5, minEllipse[i].center.y + 1.4*yCupSize),
					cv::Scalar(0, 255, 0));
			}
		}*/
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


// Apply Opencv's inbuilt surf function
void siftObjects(cv::Mat img_object, cv::Mat img_scene, std::vector<cv::KeyPoint>* keypoints_object, std::vector<cv::KeyPoint>* keypoints_scene, std::vector<cv::DMatch>* good_matches)
{
	cv::SIFT sift = cv::SIFT(40,3,0.004,10.0,1.4);
	sift.detect(img_object, *keypoints_object);
	sift.detect(img_scene, *keypoints_scene);
	//cv::FAST(img_object,*keypoints_object,5,true);
	//cv::FAST(img_scene, *keypoints_scene, 5, true);

	cv::Mat descriptors1, descriptors2;
	sift.compute(img_object, *keypoints_object, descriptors1);
	sift.compute(img_scene, *keypoints_scene, descriptors2);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match(descriptors1, descriptors2, matches);
	
	/*for (int i = 0; i < descriptors1.rows; i++)
	{
		good_matches->push_back(matches[i]);
	}*/

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	for (int i = 0; i < descriptors1.rows; i++)
	{
		if (matches[i].distance < 3*min_dist)
		{
			good_matches->push_back(matches[i]);
		}
	}
}




// Apply Opencv's inbuilt surf function
void surfObjects(cv::Mat img_object, cv::Mat img_scene, std::vector<cv::KeyPoint>* keypoints_object, std::vector<cv::KeyPoint>* keypoints_scene, std::vector<cv::DMatch>* good_matches)
{
	//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 600;

	cv::SurfFeatureDetector detector(minHessian);

	detector.detect(img_object, *keypoints_object);
	detector.detect(img_scene, *keypoints_scene);

	//-- Step 2: Calculate descriptors (feature vectors)
	cv::SurfDescriptorExtractor extractor(minHessian, 4, 2, true, false);

	cv::Mat descriptors_object, descriptors_scene;

	extractor.compute(img_object, *keypoints_object, descriptors_object);
	extractor.compute(img_scene, *keypoints_scene, descriptors_scene);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match(descriptors_object, descriptors_scene, matches);

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
	printf("-- Min dist : %f \n", min_dist);

	for (int i = 0; i < descriptors_object.rows; i++)
	{
		if (matches[i].distance <5* min_dist)
		{
			good_matches->push_back(matches[i]);
		}
	}
}

// get a CV_8UC4 (RGB) Matrix from Kinect RGB frame
cv::Mat * GetColorImage(kinectClass *kinect, BYTE* bytes, int width, int height)
{
	//const BYTE* start = (const BYTE*)bytes;
	//unsigned char* dest[COLOR_WIDTH*COLOR_HEIGHT*3];
	//float* fdest = (float*)dest;
	//long* depth2rgb = (long*)kinect->depthToRgbMap;
	//for (int j = 0; j < height; ++j) {
	//	for (int i = 0; i < width; ++i) {
	//		// Determine color pixel for depth pixel i,j
	//		long x = *depth2rgb++;
	//		long y = *depth2rgb++;
	//		// If out of bounds, then do not color this pixel
	//		if (x < 0 || y < 0 || x > width || y > height) {
	//			for (int n = 0; n < 3; ++n) *fdest++ = 0.f;
	//		}
	//		else {
	//			// Determine rgb color for depth pixel (i,j) from color pixel (x,y)
	//			const BYTE* color = start + (x + width*y) * 4;
	//			for (int n = 0; n < 3; ++n) *fdest++ = color[2 - n] / 255.f;
	//		}
	//	}
	//}



	////
	const unsigned int img_size = width*height*4;
	cv::Mat * out = new cv::Mat(height, width, CV_8UC4);
	memcpy(out->data, bytes, img_size);
	return out;
}

// get a CV_8U matrix from a Kinect depth frame 
cv::Mat * GetDepthImage(kinectClass *kinect, BYTE* depthData, NUI_LOCKED_RECT* LockedRect, int width, int height)
{
	//INuiCoordinateMapper* coordMapper;
	//kinect->sensor->NuiGetCoordinateMapper(&coordMapper);
	//// map depthpixel to depth for all depthData
	//NUI_DEPTH_IMAGE_PIXEL* depthPixelsNui = new NUI_DEPTH_IMAGE_PIXEL[(640*480)];
	//int i = 0;
	//while (i < (640*480)) {
	//	depthPixelsNui[i].depth = NuiDepthPixelToDepth(depthData[i]);
	//	depthPixelsNui[i].playerIndex = NuiDepthPixelToPlayerIndex(depthData[i]);
	//	i++;
	//}
	//NUI_COLOR_IMAGE_POINT* mappedDepthLocations = new NUI_COLOR_IMAGE_POINT[640*480];
	//coordMapper->MapDepthFrameToColorFrame(NUI_IMAGE_RESOLUTION_640x480,640*480,(NUI_DEPTH_IMAGE_PIXEL*)depthData,NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,640*480, mappedDepthLocations);

	//// now how to use NUI_COLOR_IMAGE_POINT *mappedDepthLocations
	//




	//printf("%d", mappedDepthLocations[320*240/2]);


	//cv::Mat depthImage = cv::cvarrToMat(out, false);
	//cv::namedWindow("depthtest", CV_WINDOW_AUTOSIZE);
	//cv::imshow("depthtest", depthImage);
	//cv::waitKey(0);
	//return &depthImage;

	// We run the below to map the depth image to the color image

	USHORT* data = (USHORT*)depthData;
	IplImage* out = cvCreateImage(cvSize(640, 480), 8, 1);
	int r = 480;
	int c = 640;
	long *colorCoords = new long[r*c*2];

	// Store the index into the color array corresponding to this pixel
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
		//unsigned short depth = ((*pBufS) & 0xfff8) >> 3;
		unsigned short depth = NuiDepthPixelToDepth(*pBufS);
		unsigned char intensity = depth > 0 ? 255 - (unsigned char)(256 * depth / 0x0fff) : 0;
		
		long
			x = colorCoords[i], // tried with *(pMappedBits + (i * 2)),
			y = colorCoords[i + 1]; // tried with *(pMappedBits + (i * 2) + 1);
		depthinMM[x + y*out->widthStep] = depth;
		if (x >= 0 && x < out->width && y >= 0 && y < out->height) {
			out->imageData[x + y * out->widthStep] = intensity;
		}

	}


	// Then we can simply display Iout as a mat

	
	depthImage = cv::cvarrToMat(out, true);
	//cv::Mat * depthImagePtr = &depthImage;
	//printf("%d", out[640 * 480 / 2]);
	//cv::namedWindow("colourDepth", cv::WINDOW_AUTOSIZE);
	//cv::imshow("colourDepth", depthImage);
	return &depthImage;






	//const USHORT* curr = (const USHORT*) depthData;
	//unsigned char* dest[DEPTH_WIDTH*DEPTH_HEIGHT * 3];;
	//float* fdest = (float*)dest;
	//long* depth2rgb = (long*)kinect->depthToRgbMap;
	//for (int j = 0; j < height; ++j) {
	//	for (int i = 0; i < width; ++i) {
	//		// Get depth of pixel in millimeters
	//		USHORT depth = NuiDepthPixelToDepth(*curr);
	//		// Store coordinates of the point corresponding to this pixel
	//		Vector4 pos = NuiTransformDepthImageToSkeleton(i, j, *curr);
	//		*fdest++ = pos.x / pos.w;
	//		*fdest++ = pos.y / pos.w;
	//		*fdest++ = pos.z / pos.w;
	//		// Store the index into the color array corresponding to this pixel
	//		NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
	//			NUI_IMAGE_RESOLUTION_640x480, // color frame resolution
	//			NUI_IMAGE_RESOLUTION_320x240, // depth frame resolution
	//			NULL,                         // pan/zoom of color image (IGNORE THIS)
	//			i, j, *curr,                  // Column, row, and depth in depth image
	//			depth2rgb, depth2rgb + 1        // Output: column and row (x,y) in the color image
	//			);
	//		depth2rgb += 2;
	//		*curr++;
	//	}
	//}

	//const unsigned int img_size = width*height*3;
	//cv::Mat* out = new cv::Mat(height, width, CV_8UC4);
	//memcpy(out->data, dest, img_size);
	//return out;
	//cv::Mat * inter = new cv::Mat(height, width, CV_16U);
	//cv::Mat * out = new cv::Mat(height, width, CV_8U);
	////cv::Mat * out = new cv::Mat(height, width, CV_16U);
	//std::vector<int> arr;
	//
	//// Copy image information into Mat

	//for (UINT y = 0; y < height; ++y)
	//{
	//	// Get row pointer for depth Mat
	//	USHORT* pDepthRow = inter->ptr<USHORT>(y);

	//	for (UINT x = 0; x < width; ++x)
	//	{
	//		pDepthRow[x] = depthData[y * width + x];
	//	}
	//}
	//inter->convertTo(*out, CV_8U, 255.0 / 65535);
	//return inter;
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
			kinect->m = GetColorImage(kinect,kinect->colourData, COLOR_WIDTH, COLOR_HEIGHT);
		}
		else
		{
			printf("Received colour textures pitch is zero \n");
		}

	}
	else {
		// wait for event to be triggered by Kinect (wait forever)
		WaitForSingleObject(kinect->nextDepthFrameEvent, INFINITE);

		kinect->hr = NuiImageStreamGetNextFrame(kinect->depthStream, 0, &imageFrame);
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
	}
	// We're done with the texture so unlock it
	pTexture->UnlockRect(0);

	// Release the frame
	NuiImageStreamReleaseFrame(kinect->depthStream, imageFrame);
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

cv::Point2f adjustCoords(int x, int y, int d) {
	double fx = 384.67525007;
	double fy = 312.96378221;
	double cx = 388.3476765;
	double cy = 296.26745933;

	double newx = (x - cx) * d / fx;
	double newy = (y - cy) * d / fy;

	return cv::Point2f(newx, newy);
}

void detectChessboard(cv::Mat * input, cv::Mat * output) {
	cv::Size patternsize(8, 5); //interior number of corners //source image
	cv::vector<cv::Point2f> corners; //this will be filled by the detected corners

	//CALIB_CB_FAST_CHECK saves a lot of time on images
	//that do not contain any chessboard corners
	bool patternfound = findChessboardCorners(*input, patternsize, corners,
		cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
		+ cv::CALIB_CB_FAST_CHECK);

	if (patternfound)
		cornerSubPix(*input, corners, cv::Size(11, 11), cv::Size(-1, -1),
		cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	cv::drawChessboardCorners(checkerImage, patternsize, cv::Mat(corners), patternfound);
}