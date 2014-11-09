#ifndef SHOPPINGLIST_H_
#define SHOPPINGLIST_H_

// ROS headers
#include <ros/ros.h>
// C/C++ headers
#include <iostream>
#include <utility>  // std::pair
// OpenCV headers
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
// Project headers
#include "RobustMatcher.h"

/***********************************************************************************************/

class ObjectsBag {
public:
	/* Public constructor */
	ObjectsBag() {}

	// Getters Methods
	std::vector<int> getIds() const { return ids_; }
	std::vector<std::string> getNames() const { return names_; }
	std::vector<std::vector<cv::KeyPoint> > getKeyPoints() const { return keyPoints_; }
	std::vector<cv::Mat> getDescriptors() const { return descriptors_; }
	int getTotalNumSamples() const { return totalNumSamples_; }

	// Setters Methods
	void setTotalNumSamples(const int totalNumSamples) { totalNumSamples_ = totalNumSamples; }
    
    // Public methods
	void addObject(const int id, const std::string& name, const std::vector<cv::KeyPoint>& keyPoints, const cv::Mat& descriptors);
	void clearData();

private:
	/* The total number of samples */
	int totalNumSamples_;
	/* The identifaction number of each samples */
	std::vector<int> ids_;
	/* The name of each samples */
	std::vector<std::string> names_;
	/* KeyPoints of the objects to detect */
	std::vector<std::vector<cv::KeyPoint> > keyPoints_;
	/* Descriptors of the objects to detect */
    std::vector<cv::Mat> descriptors_;
};

/***********************************************************************************************/

class ShoppingList {
public:
	/* Public constructor */
    ShoppingList(const ros::NodeHandle& nh);
    ~ShoppingList() {}

    // Getter methods
    std::vector<std::pair<int, std::string> > getResults() const { return results_; }

    // Public methods
    void trainData();
    void process(const cv::Mat& img_in);
    
private:
    /* Robust Matcher to detect the object */
    RobustMatcher rmatcher_;
    /* Objects models information */
    ObjectsBag objectsBag_;
    /* The result to embed the solution */
    std::vector<std::pair<int, std::string> > results_;
    /* Type of features detector */
    std::string detector_type_;
    /* Path to the objects models raw info */
    std::string path_to_objects_raw_;
    /* Path to the raw data */
    std::string path_to_data_;
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
