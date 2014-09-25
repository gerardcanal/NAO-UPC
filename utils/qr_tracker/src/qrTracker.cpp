#include "qrTracker.h"

QrTracker::QrTracker()
{
  scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
}

void QrTracker::track(Mat frame, int &size_x, int &size_y, string &data, Point2f &p1, Point2f &p2, Point2f &p3, Point2f &p4)
{
	Mat frame_grayscale;
  cvtColor(frame, frame_grayscale, COLOR_BGR2HSV);

	// wrap image
	int width = frame_grayscale.cols;
	int height = frame_grayscale.rows;
	uchar *raw = (uchar *)(frame_grayscale.data);
	Image image(width, height, "Y800", raw, width * height);

	//Scan the image for QRcodes
	scanner.scan(image);
	for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
    //Size
    size_x = symbol->get_location_x(2) - symbol->get_location_x(1);
    size_y = symbol->get_location_y(1) - symbol->get_location_y(0);

		//Data
		data = symbol->get_data().c_str();
		
		//Points
    if (symbol->get_location_size() == 4) {
      p1 = Point2f(symbol->get_location_x(0), symbol->get_location_y(0));
			p2 = Point2f(symbol->get_location_x(1), symbol->get_location_y(1));
			p3 = Point2f(symbol->get_location_x(2), symbol->get_location_y(2));
			p4 = Point2f(symbol->get_location_x(3), symbol->get_location_y(3));
    }else{
			p1 = Point2f(-1,-1);
			p2 = Point2f(-1,-1);
			p3 = Point2f(-1,-1);
			p4 = Point2f(-1,-1);
		}
	}
}
