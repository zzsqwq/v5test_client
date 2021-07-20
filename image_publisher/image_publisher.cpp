//
// Created by zs on 2021/7/18.
//

#include "image_publisher.h"

#include <time.h>
#include <cstdlib>
#include "ros/ros.h"
#include "v5test_client/Polygon_outerpoints.h"
#include "publisher_toolbox.h"

const float max_offset = 15;
int test_cnt = 0;
int type = 1;
float distance(cv::Point2f point_a, cv::Point2f point_b) {
  return std::sqrt(
      (point_a.x - point_b.x) * (point_a.x - point_b.x) + (point_a.y - point_b.y) * (point_a.y - point_b.y));
}
bool CheckResult(std::vector<geometry_msgs::Point32> result_points, std::vector<cv::Point2f> std_points) {
  bool flag = true;
  if (result_points.size() != std_points.size()) return false;
  for (int i = 0; i < result_points.size() && i < std_points.size(); i++) {
    cv::Point2f result_point = cv::Point(result_points[i].x, result_points[i].y);
    if (distance(result_point, std_points[i]) > max_offset) flag = false;
  }
  return flag;
}
int main(int argc, char **argv) {

  srand(unsigned(time(NULL)));
  ros::init(argc, argv, "v5test_client");

  if(argc == 2) {
    type = std::atoll(argv[1]);
  }

  ros::NodeHandle nh_;

  ros::ServiceClient client = nh_.serviceClient<v5test_client::Polygon_outerpoints>("v5test");

  v5test_client::Polygon_outerpoints image_srv;

  cv::Mat src_img;
  auto publisher_toolbox = std::make_shared<PublishToolbox>(
      cv::Scalar(0, 0, 255),
      cv::Scalar(0, 0, 255),
      6);

  std::vector<cv::Point2f> std_points;
  while (ros::ok()) {
    std_points.clear(); // clear
    if(type == 1 || type == 2)
      publisher_toolbox->GenerateImage(src_img, std_points, type);
    std::sort(std_points.begin(), std_points.end(),
              [](cv::Point2f tmp_a, cv::Point2f tmp_b) { return tmp_a.x < tmp_b.x; });

    auto img_to_send = std::make_shared<ImageInfo>(src_img, std_points);

    image_srv.request.img = *(img_to_send->img_);

    if (client.call(image_srv)) {
      if (CheckResult(image_srv.response.outer_points, std_points)) {
        test_cnt++;
        std::cout << "Has received correct answer!" << "  Correct index: " << test_cnt << std::endl;
      } else {
        test_cnt = 0;
        std::cout << "Error Answer!" << std::endl;
      }
      if (test_cnt >= 50) {
        std::cout << "Congratulations! Test pass!" << std::endl;
        return 0;
      }
    } else {
      std::cout << "Can't get Answer!" << std::endl;
    }
  }
}
