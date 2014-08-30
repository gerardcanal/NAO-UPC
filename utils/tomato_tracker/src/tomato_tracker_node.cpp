#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "tomatoTracker.h"
#include <iostream>
#include <stdio.h>

class TomatoTrackerNode
{

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  TomatoTracker tracker;
  
public:
  TomatoTrackerNode()
    : it_(nh_)
  {
		//To publish the "/image_raw" topic run: $rosrun uvc_camera uvc_camera_node 
    image_sub_ = it_.subscribe("/image_raw", 1, &TomatoTrackerNode::imageCb, this);
		cv::namedWindow("Tomatoe", cv::WINDOW_KEEPRATIO);
  }

  ~TomatoTrackerNode(){cv::destroyWindow("Tomatoe");}

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

		Point2f obj_pos;
		float area;
		tracker.track(cv_ptr->image, obj_pos, area);
		std::cout << "point: " << obj_pos.x << ", " << obj_pos.y << " radius: " << sqrt(area/M_PI) << std::endl;
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
