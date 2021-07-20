//
// Created by zs on 2021/7/18.
//

#ifndef V5TEST_CLIENT_INCLUDE_IMAGE_PUBLISH_H_
#define V5TEST_CLIENT_INCLUDE_IMAGE_PUBLISH_H_

#include <vector>

//opencv
#include <opencv2/opencv.hpp>

//ros
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "publisher_toolbox.h"

class ImageInfo {
 public:
  /**
   * @brief The constructor of ImageInfo class
   * @param img The img be sent
   * @param std_points Standard points to check answer
   */
  explicit ImageInfo(cv::Mat img, std::vector<cv::Point2f> std_points) : points_pos_(std_points) {
    image_height_ = img.rows;
    image_width_ = img.cols;
    img_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  }
 public:
  sensor_msgs::ImagePtr img_;
  int image_width_;
  int image_height_;
  std::vector<cv::Point2f> points_pos_;

};

#endif //V5TEST_CLIENT_INCLUDE_IMAGE_PUBLISH_H_
