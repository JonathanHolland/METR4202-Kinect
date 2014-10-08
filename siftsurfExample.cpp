#include "main.h"

/*
* The following is example code to run sift and/or surf
* It was not used in the final implementation as ellipse detection proved
* lightweight and efficient for this particular image recognition problem
*/

// cv::Mat img_scene = imread(); // Read in an image of the scene to sift/surf
//std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
//std::vector<cv::DMatch> good_matches;
//threshold(img_scene, img_scene, threshold_value, max_BINARY_value, threshold_type);
//threshold(img_object, img_object, threshold_value, max_BINARY_value, threshold_type);
//siftObjects(img_object, img_scene, &keypoints_object, &keypoints_scene, &good_matches);

//std::vector<cv::Point2f> obj;
//std::vector<cv::Point2f> scene;

//for (int i = 0; i < good_matches.size(); i++)
//{
//-- Get the keypoints from the good matches
//	obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
//	scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
//}

// drawing the results
//cv::namedWindow("matches", 1);
//cv::Mat img_matches;
//cv::drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches);
//imshow("matches", img_matches);

//cvWaitKey(0);

/*
* End of code used to run surf/sift functions
*/

// Apply Opencv's inbuilt surf function
void siftObjects(cv::Mat img_object, cv::Mat img_scene, std::vector<cv::KeyPoint>* keypoints_object, std::vector<cv::KeyPoint>* keypoints_scene, std::vector<cv::DMatch>* good_matches)
{
	// Very low contrast threshold is desired
	cv::SIFT sift = cv::SIFT(40, 3, 0.004, 10.0, 1.4);
	sift.detect(img_object, *keypoints_object);
	sift.detect(img_scene, *keypoints_scene);
	// Mixing descriptors and detectors/computers can allow for a better matching
	//cv::FAST(img_object,*keypoints_object,5,true);
	//cv::FAST(img_scene, *keypoints_scene, 5, true);

	cv::Mat descriptors1, descriptors2;
	sift.compute(img_object, *keypoints_object, descriptors1);
	sift.compute(img_scene, *keypoints_scene, descriptors2);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	// Brute force matcher can also be used
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
		if (matches[i].distance < 3 * min_dist)
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
		if (matches[i].distance <5 * min_dist)
		{
			good_matches->push_back(matches[i]);
		}
	}
}
