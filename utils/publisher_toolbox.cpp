//
// Created by zs on 2021/7/19.
//

#include <opencv2/opencv.hpp>
#include <cstdlib>
#include "publisher_toolbox.h"
#include "ros/package.h"

PublishToolbox::PublishToolbox(cv::Scalar line_color, cv::Scalar fill_color, int max_index) :
    line_color_(line_color), fill_color_(fill_color), max_index_(max_index), background_index_(0) {
  std::cout << "Publish Toolbox is constructed" << std::endl;
}
cv::Scalar PublishToolbox::GetLineColor() {
  cv::Scalar line_color;
  int color_chose = rand()%3;
  int color_range = rand()%150 + 70;
  switch (color_chose) {
    case 0:
      line_color = cv::Scalar(color_range,0,0);
      break;
    case 1:
      line_color = cv::Scalar(0,color_range,0);
      break;
    case 2:
      line_color = cv::Scalar(0,0,color_range);
      break;
    default:
      line_color = cv::Scalar(color_range,0,0);
      break;
  }
  return line_color;
}
void PublishToolbox::GetMinMax(std::vector<cv::Point2f> &points, cv::Point2f &point, cv::Size &size) {
  if (points.size() == 0) return;
  int min_x = points[0].x, max_x = points[0].x;
  int min_y = points[0].y, max_y = points[0].y;
  for (int i = 1; i < points.size(); i++) {
    if (min_x > points[i].x) min_x = points[i].x;
    if (max_x < points[i].x) max_x = points[i].x;
    if (min_y > points[i].y) min_y = points[i].y;
    if (max_y < points[i].y) max_y = points[i].y;
  }
  point = cv::Point2f(min_x, min_y);
  size = cv::Size(max_x - min_x, max_y - min_y);
}

bool PublishToolbox::CheckConstraint(std::vector<cv::Point2f> &vertex_points) {
  if (vertex_points.empty()) return false;
  for (int i = 0; i < vertex_points.size(); i++) {
    if (vertex_points[i].x > 950 || vertex_points[i].x < 100) return false;
    if (vertex_points[i].y > 700 || vertex_points[i].y < 100) return false;
  }
  GetMinMax(vertex_points, vertex_point_, size_);
  cv::Rect rect = cv::Rect(vertex_point_, size_);
  for (int i = 0; i < rects_.size(); i++) {
    cv::Rect andrect = rect & rects_[i];
    if (andrect.area() > 0) return false;
  }
  return true;
}
cv::Point2f PublishToolbox::GeneratePoint() {
  float x = rand() % 1024, y = rand() % 768;
  return cv::Point(x, y);
}

void PublishToolbox::GenerateContours(int edge_nums) {
  cv::Point2f init_point;
  switch (edge_nums) {
    case 0:init_point = GeneratePoint();
      radius_ = 50 + rand() % 20;
      do {
        init_point = GeneratePoint();
        contours_.clear();
        contours_.push_back(init_point);
        contours_.push_back(cv::Point(init_point.x - radius_, init_point.y - radius_));
        contours_.push_back(cv::Point(init_point.x - radius_, init_point.y + radius_));
        contours_.push_back(cv::Point(init_point.x + radius_, init_point.y + radius_));
        contours_.push_back(cv::Point(init_point.x + radius_, init_point.y - radius_));
      } while (!CheckConstraint(contours_));
      GetMinMax(contours_, vertex_point_, size_);
      rects_.push_back(cv::Rect(vertex_point_, size_));
      break;
    case 3:init_point = GeneratePoint();
      do {
        init_point = GeneratePoint();
        contours_.clear();
        contours_.push_back(init_point);
        contours_.push_back(cv::Point(init_point.x - 60, init_point.y + 80));
        contours_.push_back(cv::Point(init_point.x + 60, init_point.y + 80));
      } while (!CheckConstraint(contours_));
      GetMinMax(contours_, vertex_point_, size_);
      rects_.push_back(cv::Rect(vertex_point_, size_));
      break;
    case 4:init_point = GeneratePoint();
      do {
        init_point = GeneratePoint();
        contours_.clear();
        contours_.push_back(init_point);
        contours_.push_back(cv::Point(init_point.x + 100, init_point.y + 50));
        contours_.push_back(cv::Point(init_point.x + 100, init_point.y));
        contours_.push_back(cv::Point(init_point.x, init_point.y + 50));
      } while (!CheckConstraint(contours_));
    default:break;
  }
}

void PublishToolbox::DrawPolygon(const cv::Mat &src_img, std::vector<cv::Point2f> &poly_points, bool isFilled) {
  int k = rand() % 3;
  float x = 0, y = 0;
  switch (k) {
    case 0:GenerateContours(0);
      poly_points.push_back(contours_[0]);
      cv::circle(src_img, contours_[0], radius_, GetLineColor(), -1);
      break;
    case 1:GenerateContours(3);
      contours_int_.clear();
      for (int i = 0; i < 3; i++) {
        cv::line(src_img, contours_[i], contours_[(i + 1) % 3], GetLineColor());
      }
      poly_points.push_back(cv::Point2f(contours_[0].x, contours_[0].y + 62));
      for (int i = 0; i < 3; i++) contours_int_.push_back(cv::Point(contours_[i]));
      cv::fillConvexPoly(src_img, contours_int_, GetLineColor());
      break;
    case 2:GenerateContours(4);
      angle_ = rand() % 85;
      rotated_rect_ = cv::RotatedRect(contours_[0], cv::Size(120, 100), angle_);
      rotated_rect_.points(vertex_);
      contours_.clear();
      contours_int_.clear();
      for (int i = 0; i < 4; i++) {
        contours_.push_back(vertex_[i]);
        x += vertex_[i].x;
        y += vertex_[i].y;
      }
      if (!CheckConstraint(contours_)) {
        index_--;
        break;
      }
      for (int i = 0; i < 4; i++) {
        cv::line(src_img, vertex_[i], vertex_[(i + 1) % 4], GetLineColor());
      }
      for (int i = 0; i < 4; i++) contours_int_.push_back(cv::Point(contours_[i]));
      GetMinMax(contours_, vertex_point_, size_);
      rects_.push_back(cv::Rect(vertex_point_, size_));
      cv::fillConvexPoly(src_img, contours_int_, GetLineColor());
      poly_points.push_back(cv::Point2f(x / 4.0, y / 4.0));
      break;
    default:break;
  }
}

void PublishToolbox::GenerateImage(cv::Mat &image, std::vector<cv::Point2f> &std_points,const int &type) {
  background_index_ = (background_index_ + 1) % 5 + 1;
  if(type == 1) {
    image = cv::Mat::zeros(768, 1024, CV_8UC3);
  }
  else {
    std::string img_name =
        ros::package::getPath("v5test_client") + "/image_publisher/images/" + std::to_string(background_index_) + ".jpg";
    image = cv::imread(img_name);
  }
  image_copy_ = image;
  int poly_nums = 0;
  while (poly_nums < 3)
    poly_nums = rand() % max_index_;
  for (index_ = 0; index_ < poly_nums; index_++)
    DrawPolygon(image, std_points);
  contours_.clear();
  contours_int_.clear();
  rects_.clear();
  cv::imshow("Show image", image);
  cv::waitKey(300);

  //cv::imshow("Copy show",image);
  //std::cout<<"Show image!"<<std::endl;

}

