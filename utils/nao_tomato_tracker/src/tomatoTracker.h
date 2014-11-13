#ifndef TOMATOTRACKER_H
#define TOMATOTRACKER_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include <math.h>
#include <time.h>

class TomatoTracker
{
public:
	TomatoTracker();
	TomatoTracker(int lowerH, int lowerS, int lowerV, int upperH, int upperS, int upperV);
	void track(cv::Mat frame, cv::Point2f &obj_pos, float &radius);

private:
	int _lowerH;
	int _lowerS;
	int _lowerV;
	int _upperH;
	int _upperS;
	int _upperV;

	cv::Mat getThresholdedImage(cv::Mat imgHSV);
	void trackObject(cv::Mat imgThresh, cv::Point2f &obj_pos, float &radius);
};

#endif
