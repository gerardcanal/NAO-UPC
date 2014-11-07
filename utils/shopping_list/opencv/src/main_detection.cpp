// C++
#include <iostream>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
// NAO PROJECT
#include "robust_matcher.h"

using namespace std;
using namespace cv;

/**  GLOBAL VARIABLES  **/

int N = 4; // number of classes

const string path_img_scene = "../Data/scene_07.jpg";

const double akaze_thresh = 3e-4;    // AKAZE detection threshold set to locate about 1000 keypoints
const double ransac_thresh = 5.0f;   // RANSAC inlier threshold
const double nn_match_ratio = 0.65f; // Nearest-neighbour matching ratio
const int bb_min_inliers = 10;       // Minimal number of inliers to draw bounding box

std::string IntToString ( int Number )
{
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}


/**  Main program  **/
int main()
{

  vector<string> paths_images_objects(N);
  paths_images_objects[0] = "../Data/obj_00.jpg";
  paths_images_objects[1] = "../Data/obj_01.jpg";
  paths_images_objects[2] = "../Data/obj_02.jpg";
  paths_images_objects[3] = "../Data/obj_03.jpg";

  //Ptr<Feature2D> detector = Feature2D::create("AKAZE");
  //akaze->set("threshold", akaze_thresh);

  int nFeatures = 100000;
  Ptr<Feature2D> detector = Feature2D::create("ORB");
  detector->setInt("nFeatures", nFeatures);

  RobustMatcher rmatcher;
  rmatcher.setFeatureDetector(detector);
  rmatcher.setRatio(nn_match_ratio);

  // Open scene image
  Mat img_scene = imread(path_img_scene, IMREAD_COLOR);

  if (!img_scene.data) {
	  cout << "Could not open or find the image scene" << endl;
	  return -1;
  }

  // Open objects images and create models
  vector<Mat> images_objects(N), descriptors_objects(N);
  vector<vector<KeyPoint> > keypoints_objects(N);

  for (int i = 0; i < N; ++i)
  {
	  images_objects[i] = imread(paths_images_objects[i], IMREAD_COLOR);

	  if (!images_objects[i].data) {
	     cout << "Could not open or find image " << i << endl;
	     return -1;
	  }

	  cout << "Computing AKAZE features from object " << i << " ..." << endl;
	  (*detector)(images_objects[i], noArray(), keypoints_objects[i], descriptors_objects[i]);

  }

  // write to external file
  FileStorage fs("objects_models_ORB.yml", FileStorage::WRITE);
  fs << "numClasses" << N;

  // TOdo: GENERATE FILE WITH SOME STRUCTURE
  std::string map_name;
  for (int i = 0; i < N; ++i) {

    map_name = "Sample" + IntToString(i);
	fs << map_name << "{";

	fs << "objectId" << i;
	fs << "imageRows" << images_objects[i].rows;
	fs << "imageCols" << images_objects[i].cols;

	std::vector<cv::Point2f> keypoints_pt;
	for (int j = 0; j < keypoints_objects[i].size(); ++j)keypoints_pt.push_back(keypoints_objects[i][j].pt);

	fs << "keypoints" << cv::Mat(keypoints_pt);
	fs << "descriptors" << descriptors_objects[i];

	fs << "}";
  }

  fs.release();

  vector<DMatch> good_matches;
  vector<KeyPoint> keypoints_scene;
  rmatcher.match(img_scene, descriptors_objects, keypoints_scene, good_matches);

  vector<vector<Point2f> > points_object(N), points_scene(N);
  for( int i = 0; i < good_matches.size(); i++ )
  {
	//-- Get the keypoints from the good matches
	int imgIdx = good_matches[i].imgIdx;
	points_object[imgIdx].push_back( keypoints_objects[imgIdx][ good_matches[i].trainIdx ].pt );
	points_scene[imgIdx].push_back( keypoints_scene[ good_matches[i].queryIdx ].pt );
  }

  Mat img_matches = img_scene.clone();

  Mat inliers_mask, H;
  for (int i = 0; i < N ; ++i)
  {
	  float num_matches = static_cast<float>(points_object[i].size());
	  bool obj_found = num_matches >= 4;

	  // DEBUG INFO
	  cout << " **********************************"                           << endl;
	  cout << "Object " << i << (obj_found ? " FOUND" : " NOT FOUND")         << endl;
	  cout << "Num matches: " << num_matches                                  << endl;

	  if( !obj_found ) continue;

	  H = findHomography( Mat(points_object[i]), Mat(points_scene[i]), RANSAC, ransac_thresh, inliers_mask);

	  int num_inliers = 0;
	  for (int count = 0; count < inliers_mask.rows; ++count) if(inliers_mask.at<uchar>(count)) num_inliers++;

	  float ratio_matches = (float)num_inliers*100/num_matches;
	  obj_found = !H.empty() && (num_inliers >= bb_min_inliers) && (ratio_matches > 10.0f);

	  // DEBUG INFO
	  cout << "Num inliers: " << num_inliers                                  << endl;
	  if(obj_found) cout << "Ratio matches: " << ratio_matches << " %" << endl;

	  //-- Get the corners from the image_1 ( the object to be "detected" )
	  vector<Point2f> obj_corners(4);
	  obj_corners[0] = Point(0,0);
	  obj_corners[1] = Point( images_objects[i].cols, 0 );
	  obj_corners[2] = Point( images_objects[i].cols, images_objects[i].rows );
	  obj_corners[3] = Point( 0, images_objects[i].rows );
	  vector<Point2f> scene_corners(4);

	  if(obj_found) perspectiveTransform( obj_corners, scene_corners, H);
	  else continue;
	  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
	  line( img_matches, scene_corners[0], scene_corners[1], Scalar( 0, 255, 0), 4 );
	  line( img_matches, scene_corners[1], scene_corners[2], Scalar( 0, 255, 0), 4 );
	  line( img_matches, scene_corners[2], scene_corners[3], Scalar( 0, 255, 0), 4 );
	  line( img_matches, scene_corners[3], scene_corners[0], Scalar( 0, 255, 0), 4 );

  }

  // Create & Open Window
  namedWindow("OBJECT DETECTION", WINDOW_KEEPRATIO);

  // Show the image
  imshow("OBJECT DETECTION", img_matches);

  // Wait until key pressed
  waitKey(0);

  // Close and Destroy Window
  destroyWindow("OBJECT DETECTION");

}
