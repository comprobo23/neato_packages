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

class FixScan : public rclcpp::Node {
 public:
   FixScan() : Node("fix_scan")
   {
	 this->declare_parameter("robot_name", "");
     rclcpp::Parameter robot_name_param = this->get_parameter("robot_name");
	 robot_name = robot_name_param.as_string();
	 std::cout << "robot_name " << robot_name << std::endl;
     projector_ = new laser_geometry::LaserProjection();
     prev_cloud = 0;
     pub = this->create_publisher<sensor_msgs::msg::LaserScan>("stable_scan", 10);
     pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("projected_stable_scan", 10);
     sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&FixScan::scanCallback, this, _1));


    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

   }
 private:
    laser_geometry::LaserProjection* projector_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
    sensor_msgs::msg::PointCloud2* prev_cloud;
	std::string robot_name;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    std::shared_ptr<tf2_ros::TransformListener> listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void scanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan_in) {
	if (listener_ == 0 || projector_ == 0) {
	    return;
	}
	std::cout << "GOT A SCAN!!!" << std::endl;
	sensor_msgs::msg::LaserScan scan_mod(*scan_in);
	scan_mod.time_increment = (float)1.0/(360*5);
	scan_mod.scan_time = (float)1.0/5;
	std::string errorstr;
	if (!tf_buffer_->canTransform(
				scan_in->header.frame_id,
				robot_name + "odom",
				tf2_ros::fromMsg(scan_in->header.stamp),
				tf2::durationFromSec(1.0))) {
		std::cout << "unable to transform" << std::endl;
		return;
	}
	int best_matching_offset = -1;
	float best_match_val = -1.0;
	for (int t_offset_msecs = 200; t_offset_msecs <= 400; t_offset_msecs += 10) {
	    float match_val = 0.0;
	    sensor_msgs::msg::PointCloud2 cloud;
	    scan_mod.header.stamp = rclcpp::Time(scan_in->header.stamp) - rclcpp::Duration::from_seconds(t_offset_msecs/1000.0);
	    try {
		projector_->transformLaserScanToPointCloud(robot_name + "odom",
							   scan_mod,
							   cloud,
							   *tf_buffer_);
		if (prev_cloud == 0) {
		    prev_cloud = new sensor_msgs::msg::PointCloud2(cloud);
		    return;
		}
		// Now create iterators for fields
		sensor_msgs::PointCloud2Iterator<float> i_x(cloud, "x");
		sensor_msgs::PointCloud2Iterator<float> i_y(cloud, "y");
		sensor_msgs::PointCloud2Iterator<float> i_z(cloud, "z");
		for ( ; i_x != i_x.end(); ++i_x, ++i_y, ++i_z) {
		    float min_val = -1.0;
		    float dist;
		    sensor_msgs::PointCloud2Iterator<float> j_x(*prev_cloud, "x");
		    sensor_msgs::PointCloud2Iterator<float> j_y(*prev_cloud, "y");
		    sensor_msgs::PointCloud2Iterator<float> j_z(*prev_cloud, "z");
		    for ( ; j_x != j_x.end(); ++j_x, ++j_y, ++j_z){
			 dist = sqrt((*j_x - *i_x)*(*j_x - *i_x) +
					 (*j_y - *i_y)*(*j_y - *i_y) +
					 (*j_z - *i_z)*(*j_z - *i_z));
			 if (min_val == -1 || dist < min_val) {
			     min_val = dist;
			 }
		    }
		    match_val += min_val;
		}
		if (best_matching_offset == -1 || match_val < best_match_val) {
		    best_match_val = match_val;
		    best_matching_offset = t_offset_msecs;
		}
	    } catch (...) {
			std::cout << "exception!!!" << std::endl;
	    }
	}
	// set this based on the best matching offset
	if (best_matching_offset < 0) {
	    best_matching_offset = 0;
	}
	scan_mod.header.stamp = rclcpp::Time(scan_in->header.stamp) - rclcpp::Duration::from_seconds(best_matching_offset/1000.0);
	pub->publish(scan_mod);
	try {
	    sensor_msgs::msg::PointCloud2 final_cloud;
	    projector_->transformLaserScanToPointCloud(robot_name + "odom",
			    scan_mod,
			    final_cloud,
			    *tf_buffer_);
	    // the cloud time is 200ms ahead of the laser scan time to mimic the idea of instantaneously grabbing these points
	    final_cloud.header.stamp = rclcpp::Time(final_cloud.header.stamp) + rclcpp::Duration::from_seconds(0.2);
	    pub_cloud->publish(final_cloud);
	    delete prev_cloud;
	    prev_cloud = new sensor_msgs::msg::PointCloud2(final_cloud);
	} catch (...) {
	    std::cout << "EXCEPTION!!!" << std::endl;
	}
    }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixScan>());
  rclcpp::shutdown();
  return 0;
}
