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

	// pointer to the feature point detector object
	cv::Ptr<cv::Feature2D> detector;
	float ratio;        // max ratio between 1st and 2nd NN

  public:

	RobustMatcher() : ratio(0.65f) {

	  // AKZE is the default feature
	  detector= cv::Feature2D::create("AKAZE");

	}

	// Set the feature detector
	void setFeatureDetector(cv::Ptr<cv::Feature2D>& detect) {

	  detector= detect;
	}

	// Set the NN ratio
	void setRatio(float r) {

	  ratio= r;
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

	// Insert symmetrical matches in symMatches vector
	void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
                      const std::vector<std::vector<cv::DMatch> >& matches2,
									    std::vector<cv::DMatch>& symMatches)
	{

		// for all matches image 1 -> image 2
		for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator1= matches1.begin();
			 matchIterator1!= matches1.end(); ++matchIterator1) {

			if (matchIterator1->size() < 2) // ignore deleted matches
					continue;

			// for all matches image 2 -> image 1
			for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator2= matches2.begin();
					matchIterator2!= matches2.end(); ++matchIterator2)
			{

					if (matchIterator2->size() < 2) // ignore deleted matches
							continue;

					// Match symmetry test
					if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx  &&
						(*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx)
					{

						// add symmetrical match
						symMatches.push_back(cv::DMatch((*matchIterator1)[0].queryIdx,
														(*matchIterator1)[0].trainIdx,
														(*matchIterator1)[0].distance));
						break; // next match in image 1 -> image 2
					}
			}
		}
	}

	// Match feature points using symmetry test
	void match(const cv::Mat& image_scene,                       // input image
			   const std::vector<cv::Mat>& descriptors_objects,  // input descriptors
	           std::vector<cv::KeyPoint>& keypoints_scene,
	           std::vector<cv::DMatch>& matches)                 // output matches and keypoints

	{

		// 1a. Detection of the AKAZE features
		// 1b. Extraction of the AKAZE descriptors

		cv::Mat descriptors_scene;
		(*detector)(image_scene, cv::noArray(), keypoints_scene, descriptors_scene);

		std::cout << "Number of AKAZE points: " << keypoints_scene.size() << std::endl;
		std::cout << "Descriptor matrix size: " << descriptors_scene.rows << " by " << descriptors_scene.cols << std::endl;

		// 2. Match the two image descriptors

		// Construction of the matcher
	    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

	    /*cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1); // instantiate LSH index parameters
	    cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters
	    cv::DescriptorMatcher * matcher = new cv::FlannBasedMatcher(indexParams, searchParams);         // instantiate FlannBased matcher*/

		// from image 1 to image 2
		// based on k nearest neighbours (with k=2)
	    std::vector<std::vector<cv::DMatch> > matches1;
	    matcher->add(descriptors_objects);
	    matcher->train();
		matcher->knnMatch(descriptors_scene,
				          matches1,      // vector of matches (up to 2 per entry)
				          2);            // return 2 nearest neighbours

		/*matcher->clear();

		matcher->add(descriptors_objects);
		matcher->train();
		matcher->knnMatch(descriptors_scene,
					  matches1,      // vector of matches (up to 2 per entry)
					  2);            // return 2 nearest neighbours

		matcher->clear();*/

		/*// from image 2 to image 1
		// based on k nearest neighbours (with k=2)
		std::vector<std::vector<cv::DMatch> > matches2;
		matcher->knnMatch(descriptors_objects, descriptors_scene,
				          matches2,      // vector of matches (up to 2 per entry)
				          2);            // return 2 nearest neighbours

		std::cout << "Number of matched points 1->2: " << matches1.size() << std::endl;
		std::cout << "Number of matched points 2->1: " << matches2.size() << std::endl;*/

		// 3. Remove matches for which NN ratio is > than threshold

		// clean image 1 -> image 2 matches
		int removed= ratioTest(matches1);
		std::cout << "Number of matched points 1->2 : " << matches1.size() << std::endl;
		std::cout << "Number of matched points 1->2 (ratio test) : " << matches1.size()-removed << std::endl;
		// clean image 2 -> image 1 matches
		//removed= ratioTest(matches2);
		//std::cout << "Number of matched points 2->1 (ratio test) : " << matches2.size()-removed << std::endl;

		// 4. Remove non-symmetrical matches
		//std::vector<cv::DMatch> symMatches;
		//symmetryTest(matches1, matches2, matches);
		//std::cout << "Number of matched points (symmetry test): " << matches.size() << std::endl;

		for (int i = 0; i < matches1.size(); ++i) {
			if(matches1[i].size() > 0) matches.push_back(matches1[i][0]);
		}
	}
};

#endif /* ROBUST_MATCHER_H_ */
