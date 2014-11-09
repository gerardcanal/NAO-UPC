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
#include "nao_shopping_list/checkObjects.h"
#include "nao_shopping_list/trainObjects.h"
#include "ShoppingList.h"

using namespace std;

class ShoppingListService
{
public:
	/* Public constructor */
	ShoppingListService() : it_(nh_), shopList_(nh_)
	{
		image_sub_ = it_.subscribe("/image_raw", 1, &ShoppingListService::imageCb, this);
		grab_service_ = nh_.advertiseService("/nao_shopping_list/checkObjects", &ShoppingListService::grab, this);
		train_service_ = nh_.advertiseService("/nao_shopping_list/trainObjects", &ShoppingListService::train, this);
		cv::namedWindow("IMAGE");
	}

	~ShoppingListService() { cv::destroyAllWindows(); }

	/*  Service to grab the current image camera and check the objects */
	bool grab(nao_shopping_list::checkObjects::Request &req, nao_shopping_list::checkObjects::Response &res)
	{
		// process the current image
		shopList_.process(img_in_);

		std::vector<std::pair<int, std::string> > results = shopList_.getResults();
		res.num_objects = results.size();
		for (size_t i = 0; i < results.size(); ++i)
		{
			res.ids.push_back(results[i].first);
			res.names.push_back(results[i].second);
		}
		return true;
	}

	/* Service to train the objects in the file list */
	bool train(nao_shopping_list::trainObjects::Request &req, nao_shopping_list::trainObjects::Response &res)
	{

		cout << "Training the objects ..." << endl;
		// load the data and train
		shopList_.trainData();
		cout << "Training the objects OK" << endl;
		return true;
	}

	/* Camera callback */
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
		cv::imshow("IMAGE", img_in_);
		cv::waitKey(3);
	}

	private:
		ros::NodeHandle nh_;
		ros::ServiceServer grab_service_;
		ros::ServiceServer train_service_;
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