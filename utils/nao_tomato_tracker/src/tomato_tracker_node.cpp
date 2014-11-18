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
#include <nao_tomato_tracker/HotPlate.h>

class TomatoTrackerNode
{

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_tomato_ = nh_.advertise<geometry_msgs::Point>("/nao_tomato", 10);
  ros::Publisher pub_hot_plate_ = nh_.advertise<nao_tomato_tracker::HotPlate>("/nao_hot_plate", 10);
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  TomatoTracker tracker;
  int hsv_vals_[6];
  bool tuning_;
  
public:
  TomatoTrackerNode()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/image_raw", 1, &TomatoTrackerNode::imageCb, this);

    initParams();
    cv::namedWindow("Tomatoe", cv::WINDOW_KEEPRATIO);
    if (tuning_) 
    {
      cv::createTrackbar("lowerH", "Tomatoe", &hsv_vals_[0], 255 );
      cv::createTrackbar("upperH", "Tomatoe", &hsv_vals_[3], 255 );
      cv::createTrackbar("lowerS", "Tomatoe", &hsv_vals_[1], 255 );
      cv::createTrackbar("upperS", "Tomatoe", &hsv_vals_[4], 255 );
      cv::createTrackbar("lowerV", "Tomatoe", &hsv_vals_[2], 255 );
      cv::createTrackbar("upperV", "Tomatoe", &hsv_vals_[5], 255 );
    }
  }

  ~TomatoTrackerNode(){ cv::destroyWindow("Tomatoe"); }

  /* Debug method */
  void printHSV()
  {
    std::cout << "lowerH " << hsv_vals_[0] << std::endl;
    std::cout << "lowerS " << hsv_vals_[1] << std::endl;
    std::cout << "lowerV " << hsv_vals_[2] << std::endl;
    std::cout << "upperH " << hsv_vals_[3] << std::endl;
    std::cout << "upperS " << hsv_vals_[4] << std::endl;
    std::cout << "upperV " << hsv_vals_[5] << std::endl;
    std::cout << "*********************************" << std::endl;
  }

  /* The method set the hsv from node handle */
  void initParams()
  {
  	nh_.getParam("lowerH", hsv_vals_[0]);
  	nh_.getParam("lowerS", hsv_vals_[1]);
  	nh_.getParam("lowerV", hsv_vals_[2]);
  	nh_.getParam("upperH", hsv_vals_[3]);
  	nh_.getParam("upperS", hsv_vals_[4]);
  	nh_.getParam("upperV", hsv_vals_[5]);
    nh_.getParam("Tuning", tuning_);
  }

  /* Camera callbck */
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
    // find tomato
    cv::Mat img_out = cv_ptr->image.clone();
    cv::Point2f obj_pos;
    float area, mean;
    tracker.track(img_out, hsv_vals_, obj_pos, area, mean);

    // create point to publish and convert to meters
    geometry_msgs::Point point;
    point.x = (img_out.rows/2-obj_pos.x)/1800;
    point.y = (img_out.cols/2-obj_pos.y)/1800;
    point.z = 0;

    // create hot plate message to plublish
    nao_tomato_tracker::HotPlate hot_plate;
    hot_plate.point.x = obj_pos.x;
    hot_plate.point.y = obj_pos.y;
    hot_plate.point.z = 0;
    hot_plate.mean = area;

    // something found, then publish
    if ( area != -1 ) 
    {
        pub_tomato_.publish(point);
        pub_hot_plate_.publish(hot_plate);
    }

    // debug
    //printHSV();
    //std::cout << "point[px]: " << obj_pos.x << ", " << obj_pos.y << " radius: " << sqrt(area/M_PI) << std::endl;
    //std::cout << "d[px]: " <<img_out.rows/2-obj_pos.x << ", " << img_out.cols/2-obj_pos.y << " radius: " << sqrt(area/M_PI) << std::endl;
    //std::cout << "point[m]: " << point.x << ", " << point.y << " radius: " << sqrt(area/M_PI) << std::endl;
    cv::circle(img_out, cv::Point(obj_pos.x, obj_pos.y), 5, cv::Scalar(0, 255, 0));
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
