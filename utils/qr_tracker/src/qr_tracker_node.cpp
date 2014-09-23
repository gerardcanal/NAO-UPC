#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "qrTracker.h"
#include <iostream>
#include <stdio.h>
#include <string>

class QrTrackerNode
{

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  QrTracker tracker;
  
public:
  QrTrackerNode()
    : it_(nh_)
  {
		//To publish the "/image_raw" topic run: $rosrun uvc_camera uvc_camera_node 
    image_sub_ = it_.subscribe("/image_raw", 1, &QrTrackerNode::imageCb, this);
		cv::namedWindow("QR", cv::WINDOW_KEEPRATIO);
  }

  ~QrTrackerNode(){cv::destroyWindow("QR");}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

		//Tracker
		int size_x, size_y;
		std::string data;
	  Point2f p1, p2, p3, p4;
		tracker.track(cv_ptr->image, size_x, size_y, data, p1, p2, p3, p4);
		
		//Extra data
		float area = size_x*size_y;

		std::cout << "size_x: " << size_x << ", size_y: " << size_y 
		<< ", data: " << data
	  << ", p1: (" p1.x << ", " << p1.y << ") "
		<< ", p2: (" p2.x << ", " << p2.y << ") "
		<< ", p3: (" p3.x << ", " << p3.y << ") "
		<< ", p4: (" p4.x << ", " << p4.y << ") "
		<< std::endl;
  }
};

int main(int argc, char** argv)
{
	//TODO: Make it publisher
  ros::init(argc, argv, "qr_tracker");
  QrTrackerNode qrtrackernode;
  ros::spin();
  return 0;
}
