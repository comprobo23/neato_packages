#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include "laser_geometry/laser_geometry.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

using namespace std::chrono_literals;

class ScanProjector : public rclcpp::Node {
 public:
   ScanProjector() : Node("scan_to_pc2")
   {
     projector_ = new laser_geometry::LaserProjection();
     pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("projected_stable_scan", 10);
     sub = this->create_subscription<sensor_msgs::msg::LaserScan>("stable_scan", 10, std::bind(&ScanProjector::scanCallback, this, _1));

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

   }
 private:
    laser_geometry::LaserProjection* projector_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    std::shared_ptr<tf2_ros::TransformListener> listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void scanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan_in) {
	if (listener_ == 0 || projector_ == 0) {
	    return;
	}
	sensor_msgs::msg::PointCloud2 cloud;
	std::cout << "GOT A SCAN!!!" << std::endl;
	sensor_msgs::msg::LaserScan scan_mod(*scan_in);
	std::string errorstr;
	if (!tf_buffer_->canTransform(
				scan_in->header.frame_id,
				"odom",
				tf2_ros::fromMsg(scan_in->header.stamp),
				tf2::durationFromSec(1.0))) {
		std::cout << "unable to transform" << std::endl;
		return;
	}
	try {
		projector_->transformLaserScanToPointCloud("odom",
							   scan_mod,
							   cloud,
							   *tf_buffer_);
		pub_cloud->publish(cloud);
	} catch (...) {
		std::cout << "exception!!!" << std::endl;
	}
    }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanProjector>());
  rclcpp::shutdown();
  return 0;
}
