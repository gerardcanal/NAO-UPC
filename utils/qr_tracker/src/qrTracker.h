#ifndef QRTRACKER_H
#define QRTRACKER_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include <math.h>
#include <string>
#include <zbar.h>

class QrTracker
{
public:
    QrTracker();
    void track(cv::Mat frame, int &size_x, int &size_y, std::string &data, cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3, cv::Point2f &p4);

private:
    zbar::ImageScanner scanner;
};

#endif

