#ifndef SHOPPINGLIST_H_
#define SHOPPINGLIST_H_

// ROS
#include <ros/ros.h>
// C/C++
#include <iostream>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
// CURRENT PROJECT
#include "robust_matcher.h"

class ShoppingList {
public:
	explicit ShoppingList(const ros::NodeHandle& nh)
    {
		nh.getParam("path_to_objects_models", path_to_objects_models_);
		nh.getParam("detector_type", detector_type_);
		nh.getParam("akaze_threshold", akaze_thresh_);
		nh.getParam("orb_nFeatures", orb_nFeatures_);
		nh.getParam("ransac_threshold", ransac_thresh_);
		nh.getParam("nn_match_ratio", nn_match_ratio_);
		nh.getParam("bb_min_inliers", bb_min_inliers_);

		loadData();

		cv::Ptr<cv::Feature2D> detector = cv::Feature2D::create(detector_type_);
		if( detector_type_ == "AKAZE") detector->setDouble("threshold", akaze_thresh_);
		else if( detector_type_ == "ORB") detector->setInt("nFeatures", orb_nFeatures_);

		rmatcher_.setFeatureDetector(detector);
		rmatcher_.setRatio(nn_match_ratio_);
		rmatcher_.trainMatcher(descriptors_objects_);
    }

	~ShoppingList() {}

	void loadData()
	{
	    cv::FileStorage fs(path_to_objects_models_.c_str(), cv::FileStorage::READ);

	    fs["numClasses"] >> N_;

	    images_size_.resize(N_);
	    keypoints_objects_.resize(N_);
	    descriptors_objects_.resize(N_);

	    cv::Mat keypoints, descriptors;

	    cv::FileNode n = fs.root();
	    cv::FileNode item;
        cv::FileNodeIterator it = n.begin(), it_end = n.end();
        it++;
		for (; it != n.end(); ++it )
		{
			item = *it;

		   	int id;
	    	item["objectId"] >> id;

	    	images_size_[id].resize(2);
	    	item["imageRows"] >> images_size_[id][0];
	    	item["imageCols"] >> images_size_[id][1];

	    	item["keypoints"] >> keypoints;
	    	keypoints.copyTo(keypoints_objects_[id]);

	    	item["descriptors"] >> descriptors;
	    	descriptors.copyTo(descriptors_objects_[id]);
		}

	}

	void process(const cv::Mat& img_in)
	{
		// Matching
		std::vector<cv::DMatch> good_matches;
		std::vector<cv::KeyPoint> keypoints_scene;
		rmatcher_.match(img_in, keypoints_scene, good_matches);

		// Matching Solutions
		std::vector<std::vector<cv::Point2f> > points_object(N_), points_scene(N_);
		for( int i = 0; i < good_matches.size(); i++ )
		{
			//-- Get the keypoints from the good matches
			int imgIdx = good_matches[i].imgIdx;
			points_object[imgIdx].push_back( keypoints_objects_[imgIdx][ good_matches[i].trainIdx ] );
			points_scene[imgIdx].push_back( keypoints_scene[ good_matches[i].queryIdx ].pt );
		}

		// Estimate Homography
		cv::Mat inliers_mask, H;
		for (int i = 0; i < N_ ; ++i)
		{
		  float num_matches = static_cast<float>(points_object[i].size());
		  bool obj_found = num_matches >= 4;

		  // DEBUG INFO
		  //std::cout << " **********************************"                           << std::endl;
		  //std::cout << "Object " << i << (obj_found ? " FOUND" : " NOT FOUND")         << std::endl;
		  //std::cout << "Num matches: " << num_matches                                  << std::endl;

		  if( !obj_found ) continue;

		  H = cv::findHomography( cv::Mat(points_object[i]), cv::Mat(points_scene[i]), cv::RANSAC, ransac_thresh_, inliers_mask);

		  int num_inliers = 0;
		  for (int count = 0; count < inliers_mask.rows; ++count) if(inliers_mask.at<uchar>(count)) num_inliers++;

		  float ratio_matches = (float)num_inliers*100/num_matches;
		  obj_found = !H.empty() && (num_inliers >= bb_min_inliers_) && (ratio_matches > 10.0f);

		  // DEBUG INFO
		  //std::cout << "Num inliers: " << num_inliers                           << std::endl;
		  //if(obj_found) std::cout << "Ratio matches: " << ratio_matches << " %" << std::endl;

		  //-- Get the corners from the image_1 ( the object to be "detected" )
		  std::vector<cv::Point2f> obj_corners(4);
		  obj_corners[0] = cv::Point(0,0);
		  obj_corners[1] = cv::Point( images_size_[i][1], 0 );
		  obj_corners[2] = cv::Point( images_size_[i][1], images_size_[i][0] );
		  obj_corners[3] = cv::Point( 0, images_size_[i][0] );
		  std::vector<cv::Point2f> scene_corners(4);

		  if(obj_found)
		  {
			  //cv::perspectiveTransform( obj_corners, scene_corners, H);
			  result_.push_back(i);
		  }
		  else continue;

		}
	}

	std::vector<int> getResult() const { return result_; }

private:
	/* Robust Matcher to detect the object */
	RobustMatcher rmatcher_;
	/* The number of classes to detect */
	int N_;
	/* The result to embed the solution */
	std::vector<int> result_;
	/* The size of each image */
	std::vector<std::vector<int> > images_size_;
	/* KeyPoints of the objects to detect */
    std::vector<std::vector<cv::Point2f> > keypoints_objects_;
	/* Descriptors of the objects to detect */
	std::vector<cv::Mat> descriptors_objects_;
	/* Type of features detector */
	std::string detector_type_;
	/* Path to the objects models file */
	std::string path_to_objects_models_;
	/* AKAZE detection threshold set to locate about 1000 keypoints */
	double akaze_thresh_;
	/* ORB number of features */
	int orb_nFeatures_;
	/* RANSAC inlier threshold */
	double ransac_thresh_;
	/* Nearest-neighbour matching ratio */
	double nn_match_ratio_;
	/* Minimal number of inliers to draw bounding box */
	int bb_min_inliers_;
};


#endif
