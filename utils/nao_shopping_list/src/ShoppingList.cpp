// Project headers
#include "ShoppingList.h"

/*
*
* ObjectBag class
*
*/

/* Method to add a sample to the object */
void ObjectsBag::addObject(const int id, const std::string& name, 
		                   const std::vector<cv::KeyPoint>& keyPoints, const cv::Mat& descriptors)
{
	ids_.push_back(id);
	names_.push_back(name);
	keyPoints_.push_back(keyPoints);
	descriptors_.push_back(descriptors);
}

/* Method to clear previous data */
void ObjectsBag::clearData()
{
	ids_.clear();
	names_.clear();
	keyPoints_.clear();
	descriptors_.clear();
}


/*
*
* ShoppingList class
*
*/

/* Public constructor */
ShoppingList::ShoppingList(const ros::NodeHandle& nh)
{
	// load ROS srver parameters
    nh.getParam("path_to_objects_raw", path_to_objects_raw_);
    nh.getParam("path_to_data", path_to_data_);
    nh.getParam("detector_type", detector_type_);
    nh.getParam("akaze_threshold", akaze_thresh_);
    nh.getParam("orb_nFeatures", orb_nFeatures_);
    nh.getParam("ransac_threshold", ransac_thresh_);
    nh.getParam("nn_match_ratio", nn_match_ratio_);
    nh.getParam("ratio_min_inliers_", ratio_min_inliers_);

    // load the data
    trainData();

    // setup the detector
    cv::Ptr<cv::Feature2D> detector = cv::Feature2D::create(detector_type_);
    if( detector_type_ == "AKAZE") detector->setDouble("threshold", akaze_thresh_);
    else if( detector_type_ == "ORB") detector->setInt("nFeatures", orb_nFeatures_);

    // setup the matcher
    rmatcher_.setFeatureDetector(detector);
    rmatcher_.setRatio(nn_match_ratio_);
    rmatcher_.trainMatcher(objectsBag_.getDescriptors());
}

/* Method to load data and train */
void ShoppingList::trainData()
{
	/* Cleat previous data */
	objectsBag_.clearData();

	/* Load YAML file to read images and extract keypoints and descriptors */
	cv::FileStorage fs(path_to_objects_raw_.c_str(), cv::FileStorage::READ);

	// get the number of object classes
	int N; fs["num_classes"] >> N;

	// iterate over the objects
	int idx = 1, count = 1;
	cv::FileNode objects = fs["objects"];
	cv::FileNodeIterator it = objects.begin(), it_end = objects.end();
	for ( ; it != it_end; ++it, idx++ )
	{
		// debug info
		std::cout << "Loading Object " << idx << std::endl;

		// gettting object properties
		int id = (int)(*it)["object_id"];
		std::string name = (std::string)(*it)["object_name"];

		// iterate over the object images
		cv::FileNode samples = (*it)["samples"];
		cv::FileNodeIterator it2 = samples.begin(), it2_end = samples.end();
		for ( ; it2 != it2_end; ++it2, count++ )
		{
			/* Loads the image */
			cv::Mat img_sample = cv::imread( path_to_data_+(std::string)(*it2), cv::IMREAD_COLOR);

			std::vector<cv::KeyPoint> keypoints;
			cv::Mat descriptors;
			/* Compute the keypoints and descriptors */
			rmatcher_.computeKeyPointsAndDescriptors(img_sample, keypoints, descriptors);
			/* Adds the object to the list */
			objectsBag_.addObject(id, name, keypoints, descriptors);
			// debug info
			std::cout << "Num. descriptors " << descriptors.size() << std::endl;
		}

	}
	// set the counted number of total object samples
	objectsBag_.setTotalNumSamples(count);
	std::cout << "Loading DONE " << std::endl;
}

/* Method to process the image and detect objects */
void ShoppingList::process(const cv::Mat& img_in)
{
    // Clear previous results
    results_.clear();

    // Descriptors Matching
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::KeyPoint> keypoints_scene;
    rmatcher_.match(img_in, keypoints_scene, good_matches);

    // Getting the number of samples
    int N = objectsBag_.getTotalNumSamples();

    // Getting the objects ids and names
    std::vector<int> ids = objectsBag_.getIds();
	std::vector<std::string> names = objectsBag_.getNames();

    // Getting the objects models keypoints
    std::vector<std::vector<cv::KeyPoint> > keypoints_objects = objectsBag_.getKeyPoints();

    // Matching Solutions
    std::vector<std::vector<cv::Point2f> > points_object(N), points_scene(N);
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        int imgIdx = good_matches[i].imgIdx;
        points_object[imgIdx].push_back( keypoints_objects[imgIdx][ good_matches[i].trainIdx ].pt );
        points_scene[imgIdx].push_back( keypoints_scene[ good_matches[i].queryIdx ].pt );
    }

    // Estimate Homography
    cv::Mat inliers_mask, H;
    for (int i = 0; i < N ; ++i)
    {
    	// check nummber of found matches
		float num_matches = static_cast<float>(points_object[i].size());
		
		// check at least 4 points to consider an object
		bool obj_found = num_matches >= 4;
		if( !obj_found ) 
		{			
			// DEBUG INFO
			std::cout << " **********************************"                           << std::endl;
			std::cout << "Object " << i << (obj_found ? " FOUND" : " NOT FOUND")         << std::endl;
			std::cout << "Matches " << num_matches                                       << std::endl;
			continue;
		}

		// compute homography
		H = cv::findHomography( cv::Mat(points_object[i]), cv::Mat(points_scene[i]), cv::RANSAC, ransac_thresh_, inliers_mask);

		// extract the inliers 
		int num_inliers = 0;
		std::vector<cv::KeyPoint> keypoints_inliers;
		for (int count = 0; count < inliers_mask.rows; ++count) if(inliers_mask.at<uchar>(count)) num_inliers++;

		// compute the ratio between found matches and inliers
		float ratio_matches = (float)num_inliers*100/num_matches;
		// rule to decide if is an object
		obj_found = !H.empty() && (ratio_matches > ratio_min_inliers_);

		// DEBUG INFO
		std::cout << " **********************************"                                          << std::endl;
		std::cout << "Matches " << num_matches                                                      << std::endl;
		std::cout << "Object " << i << " - " << names[i] << (obj_found ? " FOUND" : " NOT FOUND")   << std::endl;
		std::cout << "Num matches: " << num_matches                                                 << std::endl;
		std::cout << "Num inliers: " << num_inliers                                                 << std::endl;
		std::cout << "Ratio matches: " << ratio_matches << " %"                                     << std::endl;

		// save the results
		if(obj_found) results_.push_back( std::make_pair(ids[i], names[i]) );

		else continue;
    }
}