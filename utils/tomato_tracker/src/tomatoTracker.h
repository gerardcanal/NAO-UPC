#ifndef TOMATOTRACKER_H
#define TOMATOTRACKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include <math.h>
#include <time.h>

using namespace cv;

class TomatoTracker
{
public:
	TomatoTracker();
	TomatoTracker(int lowerH, int lowerS, int lowerV, int upperH, int upperS, int upperV);
	void track(Mat frame, Point2f &obj_pos, float &area);

private:
	int _lowerH;
	int _lowerS;
	int _lowerV;
	int _upperH;
	int _upperS;
	int _upperV;

	Mat getThresholdedImage(Mat imgHSV);
	void trackObject(Mat imgThresh, Point2f &obj_pos, float &area);
};

#endif
