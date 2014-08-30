#include "tomatoTracker.h"

TomatoTracker::TomatoTracker()
{
	_lowerH = 0;
	_lowerS = 100;
	_lowerV = 200;
	_upperH = 180;
	_upperS = 256;
	_upperV = 256;
}

TomatoTracker::TomatoTracker(int lowerH, int lowerS, int lowerV, int upperH, int upperS, int upperV)
{
	_lowerH = lowerH;
	_lowerS = lowerS;
	_lowerV = lowerV;
	_upperH = upperH;
	_upperS = upperS;
	_upperV = upperV;
}

void TomatoTracker::track(Mat frame, Point2f &obj_pos, float &area)
{
	Mat imgHSV;
	//cvtColor(frame, imgHSV, CV_BGR2HSV); 
	cvtColor(frame, imgHSV, COLOR_BGR2HSV);

	Mat imgThresh = getThresholdedImage(imgHSV);

	blur(imgThresh, imgThresh, Size(3,3));

	trackObject(imgThresh, obj_pos, area);
}

Mat TomatoTracker::getThresholdedImage(Mat imgHSV)
//This function threshold the HSV image and create a binary image
{
	Mat imgThresh;
	inRange(imgHSV, Scalar(_lowerH, _lowerS, _lowerV), Scalar(_upperH, _upperS, _upperV), imgThresh);
	return imgThresh;
}

 
void TomatoTracker::trackObject(Mat imgThresh, Point2f &obj_pos, float &area)
//This function tracks the object with bigger area
{
  //Get the moments
  Moments mu = moments(imgThresh, true); 

  //Get the mass center:
  Point2f mc;
  if (mu.m00 > 2000){//if area<2000 is noise
  	mc = Point2f(static_cast<float>(mu.m10/mu.m00) , static_cast<float>(mu.m01/mu.m00)); 
  }else{
  	mc = Point2f(static_cast<float>(-1),static_cast<float>(-1));
  }

	obj_pos = mc;
  area = mu.m00;
}
