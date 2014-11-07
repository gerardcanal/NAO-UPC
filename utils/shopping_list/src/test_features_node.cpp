// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// C/C++
#include <iostream>
// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

class TestFeatures
{
public:
    TestFeatures() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/nao_camera/image_raw", 1, &TestFeatures::imageCb, this);
        image_pub_ = it_.advertise("/image_keypoints", 1);

        detector_ = cv::Feature2D::create("AKAZE");
        detector_->setDouble("threshold", 3e-4);

    }

    ~TestFeatures() { }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // ROS message to OpenCV Mat
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

        // Compute keypoints
        std::vector<cv::KeyPoint> keypoints_scene;
        cv::Mat descriptors_scene, img_out;
        (*detector_)(img_in_.clone(), cv::noArray(), keypoints_scene, descriptors_scene);

        // Draw Keypoints
        cv::drawKeypoints(img_in_.clone(), keypoints_scene, img_out);

        // OpenCV Mat to ROS
        img_out.copyTo(cv_ptr->image);
        sensor_msgs::ImagePtr image_msg = cv_ptr->toImageMsg();

        // Publish message
        image_pub_.publish(image_msg);

    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Ptr<cv::Feature2D> detector_;
    cv::Mat img_in_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shopping_list");
  TestFeatures test;
  ros::spin();
  return 0;
}




