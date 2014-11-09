/*
 * robust_matcher.h
 *
 *  Created on: Aug 23, 2014
 *      Author: eriba
 */

#ifndef ROBUST_MATCHER_H_
#define ROBUST_MATCHER_H_

/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 9 of the cookbook:
   Computer Vision Programming using the OpenCV Library.
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify,
   and distribute this source code, or portions thereof, for any purpose, without fee,
   subject to the restriction that the copyright notice may not be removed
   or altered from any source or altered source distribution.
   The software is released on an as-is basis and without any warranties of any kind.
   In particular, the software is not guaranteed to be fault-tolerant or free from failure.
   The author disclaims all warranties with regard to this software, any use,
   and any consequent failure, is purely the responsibility of the user.

   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

class RobustMatcher {

  private:

	/*  Pointer to the feature point detector */
	cv::Ptr<cv::Feature2D> detector_;
	/*  Pointer to the feature matcher */
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    /*  Max ratio between 1st and 2nd NN */
	float ratio;

	/*cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1); // instantiate LSH index parameters
	cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters
	cv::DescriptorMatcher * matcher = new cv::FlannBasedMatcher(indexParams, searchParams);         // instantiate FlannBased matcher*/


  public:

	RobustMatcher() : ratio(0.65f) {

	  // AKAZE is the default feature
	  detector_= cv::Feature2D::create("AKAZE");
	  // Brute Force matcher with L2 is the default matcher
      matcher_= cv::DescriptorMatcher::create("BruteForce-Hamming");
	}

	// Set the feature detector
	void setFeatureDetector(cv::Ptr<cv::Feature2D>& detect) {

	  detector_= detect;
	}

	// Set the NN ratio
	void setRatio(float r) {

	  ratio= r;
	}

	// Compute the keypoints and the descriptors of a given image
	void computeKeyPointsAndDescriptors( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
	{
		(*detector_)(image, cv::noArray(), keypoints, descriptors);
	}

	// Add models descriptors and train matcher
	void trainMatcher(const std::vector<cv::Mat>& descriptors_objects) {
		matcher_->add(descriptors_objects);
	    matcher_->train();
	}

	// Clear matches for which NN ratio is > than threshold
	// return the number of removed points
	// (corresponding entries being cleared, i.e. size will be 0)
	int ratioTest(std::vector<std::vector<cv::DMatch> >& matches) {

	int removed=0;

		// for all matches
		for (std::vector<std::vector<cv::DMatch> >::iterator matchIterator= matches.begin();
		matchIterator!= matches.end(); ++matchIterator) {

			// if 2 NN has been identified
			if (matchIterator->size() > 1) {
				// check distance ratio
				if ((*matchIterator)[0].distance/(*matchIterator)[1].distance > ratio) {
					matchIterator->clear(); // remove match
					removed++;
				}
			} else { // does not have 2 neighbours
				matchIterator->clear(); // remove match
				removed++;
			}
		}
	return removed;
    }

	// Match feature points using symmetry test
	void match(const cv::Mat& image_scene,                       // input image
	           std::vector<cv::KeyPoint>& keypoints_scene,
	           std::vector<cv::DMatch>& matches)                 // output matches and keypoints

	{

		// 1a. Detection of the AKAZE features
		// 1b. Extraction of the AKAZE descriptors
		cv::Mat descriptors_scene;
	    (*detector_)(image_scene, cv::noArray(), keypoints_scene, descriptors_scene);


		// 2. Match the two image descriptors

		// from image 1 to image 2
		// based on k nearest neighbours (with k=2)
	    std::vector<std::vector<cv::DMatch> > matches1;
		matcher_->knnMatch(descriptors_scene,
				          matches1,      // vector of matches (up to 2 per entry)
				          2);            // return 2 nearest neighbours


		// 3. Remove matches for which NN ratio is > than threshold

		// clean image 1 -> image 2 matches
		int removed= ratioTest(matches1);

		for (int i = 0; i < matches1.size(); ++i) {
			if(matches1[i].size() > 0) matches.push_back(matches1[i][0]);
		}
	}
};

#endif /* ROBUST_MATCHER_H_ */

