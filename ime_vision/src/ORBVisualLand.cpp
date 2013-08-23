/*
 * ORBVisualLand.cpp
 *
 *  Created on: Jul 31, 2013
 *      Author: barcelosandre
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/nonfree/features2d.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW1[] = "RGB Image ORB Feature";
static const char WINDOW2[] = "Depth Image ORB Feature";

class Ime_Vision_Core
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber rgb_image_sub_;
  image_transport::Publisher rgb_image_pub_;
  image_transport::Subscriber depth_image_sub_;
  image_transport::Publisher depth_image_pub_;


  cv::OrbFeatureDetector orbDetector;
  std::vector<cv::KeyPoint> keypoints;
  /*
   ORB parameters
	WTA_K = 2
	edgeThreshold = 31
	firstLevel = 0
	nFeatures = 500
	nLevels = 8
	patchSize = 31
	scaleFactor = 1.20000004768
	scoreType = 0
   */

  cv::Mat output;

  cv::OrbDescriptorExtractor orbExtractor;
  cv::Mat descriptors;

public:
  Ime_Vision_Core()
    : it_(nh_)
  {
    rgb_image_pub_ = it_.advertise("/ime_vision/features/rgb_image", 1);
    rgb_image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Ime_Vision_Core::rgbImageCallBack, this);

    depth_image_pub_ = it_.advertise("/ime_vision/features/depth_image", 1);
    depth_image_sub_ = it_.subscribe("/camera/depth/image", 1, &Ime_Vision_Core::depthImageCallBack, this);

    //orbDetector.setInt("nFeatures",);
    //cv::namedWindow(WINDOW1);
    //cv::namedWindow(WINDOW2);
  }

  ~Ime_Vision_Core()
  {
	cv::destroyWindow(WINDOW1);
    cv::destroyWindow(WINDOW2);
  }

  void rgbImageCallBack(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    orbDetector.detect(cv_ptr->image, this->keypoints);
    orbExtractor.compute(cv_ptr->image, this->keypoints, this->descriptors);

    if (keypoints.size() > 0)
    		ROS_INFO("x: %f,y: %f",keypoints[keypoints.size()-1].pt.x,keypoints[keypoints.size()-1].pt.y);

    cv::drawKeypoints(cv_ptr->image, this->keypoints, this->output);

    cv::imshow(WINDOW1, this->output);
    cv::waitKey(1);

    rgb_image_pub_.publish(cv_ptr->toImageMsg());

    keypoints.clear();
  }

	void depthImageCallBack(const sensor_msgs::ImageConstPtr & msg) {
		cv_bridge::CvImagePtr cv_ptr_depth;
		cv::Mat depth_frame;

		try {
			cv_ptr_depth = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
		} catch (cv_bridge::Exception & e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

    	depth_frame = cv_ptr_depth->image;

    	//ROS_INFO("%f\n", depth_frame.at<float>(240,320));

    	/*for(int i=50; i<depth_frame.rows; i++){
    	   for(int j=50; j<depth_frame.cols; j++){
    		   //depth_frame.data[depth_frame.step[0]*i + depth_frame.step[1]* j + 0] = 250;
    		   //depth_frame.data[depth_frame.step[0]*i + depth_frame.step[1]* j + 1] = 250;
    		   //depth_frame.data[depth_frame.step[0]*i + depth_frame.step[1]* j + 2] = 250;

    		   printf("%u\n", depth_frame.data[depth_frame.step[0]*i + depth_frame.step[1]* j + 0]);

    		   depth_frame.data[depth_frame.step[0]*i + depth_frame.step[1]* j + 1];
    		   depth_frame.data[depth_frame.step[0]*i + depth_frame.step[1]* j + 2];
    	    }
    	}*/

		cv::imshow(WINDOW2, depth_frame);
		cv::waitKey(1);

		depth_image_pub_.publish(cv_ptr_depth->toImageMsg());
}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ORB_Feature");
  ROS_INFO("IME_VISION Initialized!");
  Ime_Vision_Core ic;
  ros::spin();
  return 0;
}


