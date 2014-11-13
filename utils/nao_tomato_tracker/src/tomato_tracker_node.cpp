// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
// C/C++
#include <iostream>
#include <stdio.h>
// Current Project
#include "tomatoTracker.h"

class TomatoTrackerNode
{

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_ = nh_.advertise<geometry_msgs::Point>("/nao_tomato", 10);
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  TomatoTracker tracker;
  
public:
  TomatoTrackerNode()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/image_raw", 1, &TomatoTrackerNode::imageCb, this);
	cv::namedWindow("Tomatoe", cv::WINDOW_KEEPRATIO);
  }

  ~TomatoTrackerNode(){ cv::destroyWindow("Tomatoe"); }

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
    	// fing tomato
    	cv::Mat img_out = cv_ptr->image.clone();
		cv::Point2f obj_pos;
		float area;
		tracker.track(img_out, obj_pos, area);

		// create point to publish
		geometry_msgs::Point point;
		point.x = obj_pos.x;
		point.y = obj_pos.y;
		point.z = 0;
		// tomato found, then publish
		if ( area != -1 ) pub_.publish(point);

		// debug
		std::cout << "point: " << obj_pos.x << ", " << obj_pos.y << " radius: " << sqrt(area/M_PI) << std::endl;
		cv::circle(img_out, cv::Point(point.x, point.x), 5, cv::Scalar(0, 255, 0));
		cv::imshow("Tomatoe", img_out);
		cv::waitKey(3);		
  }
};

int main(int argc, char** argv)
{
	//TODO: Make it publisher
  ros::init(argc, argv, "tomato_tracker");
  TomatoTrackerNode tomatotrackernode;
  ros::spin();
  return 0;
}
