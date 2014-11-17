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
	TomatoTracker() {}
	void track(cv::Mat &frame, int hsv_values[6], cv::Point2f &obj_pos, float &radius, float &mean);

private:
	cv::Mat getThresholdedImage(const cv::Mat &imgHSV, int hsv_values[6]);
    float getMean(const cv::Mat &frame, const cv::Rect &boundingRect);
	void trackObject(const cv::Mat &imgThresh, cv::Point2f &obj_pos, float &radius, cv::Rect &boundingRect);
};

struct MyStruct
{
    int idx;
    double area;

    MyStruct(int i, double a) : idx(i), area(a) {}
};

struct greater_than_key
{
    inline bool operator() (const MyStruct& struct1, const MyStruct& struct2)
    {
        return (struct1.area > struct2.area);
    }
};

#endif
