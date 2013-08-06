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

static const char WINDOW[] = "ORBVisualLand";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  cv::OrbFeatureDetector orbDetector;
  std::vector<cv::KeyPoint> keypoints;

  cv::Mat output;

  cv::OrbDescriptorExtractor orbExtractor;
  cv::Mat descriptors;



public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    cv::drawKeypoints(cv_ptr->image, this->keypoints, this->output);

    cv::imshow(WINDOW, this->output);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ORB_Feature_Detector");
  ImageConverter ic;
  ros::spin();
  return 0;
}


