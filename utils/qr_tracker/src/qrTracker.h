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

using namespace cv;
using namespace std;
using namespace zbar;

class QrTracker
{
public:
	QrTracker();
	void track(Mat frame, int &size_x, int &size_y, string &data, Point2f &p1, Point2f &p2, Point2f &p3, Point2f &p4);

private:
	ImageScanner scanner;
};

#endif
