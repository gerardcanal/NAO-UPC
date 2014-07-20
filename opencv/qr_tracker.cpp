#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <zbar.h>
#include <string>
#include <iostream>
#include <time.h>

using namespace cv;
using namespace std;
using namespace zbar;

#define FONT FONT_HERSHEY_PLAIN
#define FONT_COLOR_DATA	Scalar(255,0,255)
#define LINE_TYPE CV_AA
#define VIDEOFPS 7.5
#define VIDEOPATH "video/qr_tracker.avi"


int main(){
	VideoCapture cap(0); // open the default camera
    	if(!cap.isOpened()){
		printf("Capture failure\n"); 
        	return -1;
	}
	namedWindow("QR", CV_WINDOW_KEEPRATIO);

    	ImageScanner scanner;
    	scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

	//Frame rate variables
	time_t start,end;
	time(&start);
	int counterTime=0;

	//iterate through each frames of the video
	Mat frame;
	cap >> frame;
	VideoWriter outputVideo(VIDEOPATH, CV_FOURCC('D','I','V','X'), VIDEOFPS, frame.size(), true);
	while(true){
		// Convert to grayscale
		Mat frame_grayscale;
        	cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

		// wrap image
		int width = frame_grayscale.cols;
		int height = frame_grayscale.rows;
		uchar *raw = (uchar *)(frame_grayscale.data);
		Image image(width, height, "Y800", raw, width * height);

		// Scan the image for QRcodes
		scanner.scan(image);
		for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
		    //Size
		    int size_x = symbol->get_location_x(2) - symbol->get_location_x(1);
		    int size_y = symbol->get_location_y(1) - symbol->get_location_y(0);
		    putText(frame, format("Size: %ix%i", size_x, size_y), cvPoint(10, frame.rows-70), FONT, 2, FONT_COLOR_DATA, 1.5, LINE_TYPE);
		    //Data
		    putText(frame, format("Data: %s", symbol->get_data().c_str()), cvPoint(10, frame.rows-30), FONT, 2, FONT_COLOR_DATA, 1.5, LINE_TYPE);
		    // Location
		    if (symbol->get_location_size() == 4) {
		        line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(255,0,0), 2, 8, 0);
		        line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0,255,0), 2, 8, 0);
		        line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0,0,255), 2, 8, 0);
		        line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0,255,255), 2, 8, 0);
		    }
		}
		
		//Print frame rate
		time(&end);
		++counterTime;
		double sec=difftime(end,start);
		double fps=counterTime/sec;
		putText(frame, format("FPS: %lf",fps), cvPoint(frame.cols-180, 40), FONT, 2, FONT_COLOR_DATA, 1.5, LINE_TYPE);

		imshow("QR", frame);
		outputVideo.write(frame);
		cap >> frame;

		//Wait 80mS
		int c = cvWaitKey(80);
		//If 'ESC' is pressed, break the loop
		if((char)c==27 ) break;
	}
	return 0;
}
