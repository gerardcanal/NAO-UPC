// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// C/C++
#include <iostream>
// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
// CURRENT PROJECT
#include "shopping_list/checkObjects.h"
#include "ShoppingList.h"

class ShoppingListService
{
public:
    ShoppingListService() : it_(nh_), shopList_(nh_)
	{
		image_sub_ = it_.subscribe("/image_raw", 1, &ShoppingListService::imageCb, this);
		grab_service_ = nh_.advertiseService("/shopping_list/checkObjects", &ShoppingListService::grab, this);
		//cv::namedWindow("IMAGE");
	}

    ~ShoppingListService() { /*cv::destroyAllWindows();*/ }

    bool grab(shopping_list::checkObjects::Request &req, shopping_list::checkObjects::Response &res)
    {

      img_in_ = cv::imread("/home/eriba/eclipse_ws/nao_buy_list/Data/scene_07.jpg", cv::IMREAD_COLOR);

      shopList_.process(img_in_);

      std::vector<int> result = shopList_.getResult();
      res.num_objects = result.size();
      for (size_t i = 0; i < result.size(); ++i) res.shopping_list.push_back(result[i]);

      return true;
    }

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
        cv_ptr->image.copyTo(img_in_);
        //cv::imshow("IMAGE", img_in_);
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer grab_service_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ShoppingList shopList_;
    cv::Mat img_in_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shopping_list");
  ShoppingListService service;
  ros::spin();
  return 0;
}




