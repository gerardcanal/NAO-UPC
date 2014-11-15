#include "tomatoTracker.h"

/* This function finds the tomato and return the position as well as the area */
void TomatoTracker::track(cv::Mat &frame, int hsv_values[6], cv::Point2f &obj_pos, float &radius)
{
	cv::Mat imgHSV;
	cv::cvtColor(frame, imgHSV, cv::COLOR_BGR2HSV); 

	cv::Mat imgThresholded = this->getThresholdedImage(imgHSV, hsv_values);

	// Morphological opening (remove small objects from the foreground)
	cv::erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(25, 25)));
	
	// Morphological closing (fill small holes in the foreground)
	//cv::dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	//cv::erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	this->trackObject(imgThresholded, obj_pos, radius);
}

/* This function threshold the HSV image and create a binary image */
cv::Mat TomatoTracker::getThresholdedImage(const cv::Mat &imgHSV, int hsv_values[6])
{
	cv::Mat imgThresholded;
	cv::Scalar minHSV(hsv_values[0], hsv_values[1], hsv_values[2]);
	cv::Scalar maxHSV(hsv_values[3], hsv_values[4], hsv_values[5]);
	cv::inRange(imgHSV, minHSV, maxHSV, imgThresholded);
	return imgThresholded;
}

/* This function tracks the object with bigger area */
void TomatoTracker::trackObject(const cv::Mat &imgThresholded, cv::Point2f &obj_pos, float &radius)
{
	// Find circles
	/*std::vector<cv::Vec3f> circles;
    cv::HoughCircles(imgThresholded, circles, cv::HOUGH_GRADIENT, 2, imgThresholded.rows/4, 200, 100 );
	std::cout << circles.size() << std::endl;*/

	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( imgThresholded.clone(), contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	
	// Compute areas and prepare it to sort
	std::vector<MyStruct> areas;
	for(int i = 0; i < contours.size(); ++i)
	{
		std::vector<cv::Point> cnt = contours[i], cnt_approx;
		double cnt_len = cv::arcLength(cnt, true);
		cv::approxPolyDP(cnt, cnt_approx, 0.02*cnt_len, true);
		double area = contourArea(cnt_approx);
		areas.push_back(MyStruct(i, area));
	}

	// sort by area
	std::sort(areas.begin(), areas.end(), greater_than_key());

	// get contour with higher area
	std::vector<cv::Point> cnt = contours[areas[0].idx];

	//Get the moments
	cv::Moments mu = cv::moments(cnt, true); 

	//Get the mass center:
	if (mu.m00 > 2000) { //if radius<2000 is noise
		obj_pos = cv::Point2f(static_cast<float>(mu.m10/mu.m00) , static_cast<float>(mu.m01/mu.m00));
		radius = mu.m00;
	} else {
		obj_pos = cv::Point2f(-1, -1);
		radius = -1;
	}
}
