//
// Created by zs on 2021/7/19.
//

#ifndef V5TEST_CLIENT_UTILS_PUBLISHER_TOOLBOX_H_
#define V5TEST_CLIENT_UTILS_PUBLISHER_TOOLBOX_H_

#include <iostream>
#include <vector>
#include <time.h>
#include <cstdlib>
//opencv
#include <opencv2/opencv.hpp>
//ros
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

class PublishToolbox {
 public:
  /**
   * @brief The constructor of PublishToolbox class
   * @param line_color The color of draw Polygon used
   * @param fill_color The color of fill Polygon used
   * @param max_index The max index of Polygon numbers
   */
  explicit PublishToolbox(cv::Scalar line_color, cv::Scalar fill_color, int max_index);
  /**
   * @brief The constraint checker of Polygon
   * @param vertex_points Polygon vertex
   * @return If meet the constraint, true is satisfy
   */
  bool CheckConstraint(std::vector<cv::Point2f> &vertex_points);
  /**
   * @brief Generate an image with Polygon
   * @param image The image reference
   * @param std_points Standard points to check answer
   */
  void GenerateImage(cv::Mat &image, std::vector<cv::Point2f> &std_points,const int &type=1);
  /**
   * @brief The function to draw Polygon in base image
   * @param src_img Image to draw
   * @param poly_points Same as std_points
   * @param isFilled If fill the Polygon, true is filled it
   */
  void DrawPolygon(const cv::Mat &src_img, std::vector<cv::Point2f> &poly_points, bool isFilled = true);
  /**
   * @brief Generate the Polygon vertex
   * @param edge_nums The number of sides of the generated Polygon
   */
  void GenerateContours(int edge_nums = 0);
  /**
   * @brief Generate initial_point
   * @return The initial_point is generated
   */
  cv::Point2f GeneratePoint();
  /**
   * @brief Get min_x min_y max_x min_y to check constraint
   * @param points Polygon vertex
   * @param point The point (min_x,min_y)
   * @param size The size (max_x-min_x,max_y-min_y)
   */
  void GetMinMax(std::vector<cv::Point2f> &points, cv::Point2f &point, cv::Size &size);
  /**
   * @brief Get color of Polygon
   * @return Line color
   */
  cv::Scalar GetLineColor();
 private:
  //! init_point_ of Polygon
  cv::Point2f init_point_;
  //! shallow image copy of source img
  cv::Mat image_copy_;
  //! vertex of Polygon
  std::vector<cv::Point2f> contours_;
  //! int type vertex
  std::vector<cv::Point> contours_int_;
  //! line color to draw Polygon
  cv::Scalar line_color_;
  //! color to fill Polygon
  cv::Scalar fill_color_;
  //! RotateRect to check constraint
  cv::RotatedRect rotated_rect_;
  //! Point2f* type vertex
  cv::Point2f vertex_[4];
  //! rects has been draw
  std::vector<cv::Rect> rects_;
  //! vertex_point of Rect
  cv::Point2f vertex_point_;
  //! Rect size
  cv::Size size_;
  //! the max index of picture numers
  int max_index_; // picture index
  //! image index to done
  int index_;
  //! background image index
  int background_index_;
  //! rotatedrect angle
  int angle_;
  //! circle radius
  int radius_;
};

#endif //V5TEST_CLIENT_UTILS_PUBLISHER_TOOLBOX_H_
