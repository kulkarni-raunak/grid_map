/*
 * grid_map_pcl_pointcloud_to_gridmap_node.cpp
 */

#include <pcl_conversions/pcl_conversions.h>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

namespace gm = ::grid_map::grid_map_pcl;

class GridMapPclPointcloudToGridMapNode : public rclcpp::Node
{
public:
  GridMapPclPointcloudToGridMapNode()
  : Node("grid_map_pcl_pointcloud_to_gridmap_node"),
    grid_map_pcl_loader_(this->get_logger())
  {
    pointcloud_topic_ = getOrDeclareParameter<std::string>("pointcloud_topic", "/points");
    grid_map_topic_ = getOrDeclareParameter<std::string>("grid_map_topic", "/elevation_map");
    map_frame_ = getOrDeclareParameter<std::string>("map_frame", "");
    map_layer_name_ = getOrDeclareParameter<std::string>("map_layer_name", "elevation");
    pcl_config_file_ = getOrDeclareParameter<std::string>("pcl_config_file", gm::getParameterPath());
    queue_size_ = getOrDeclareParameter<int>("queue_size", 1);

    grid_map_pcl_loader_.loadParameters(pcl_config_file_);

    auto publisher_qos = rclcpp::QoS(1).transient_local();
    grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      grid_map_topic_, publisher_qos);

    auto subscription_qos = rclcpp::SensorDataQoS().keep_last(queue_size_);
    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, subscription_qos,
      std::bind(
        &GridMapPclPointcloudToGridMapNode::pointcloudCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to '%s' and publishing GridMap on '%s'.",
      pointcloud_topic_.c_str(),
      grid_map_topic_.c_str());
  }

private:
  template<typename ParameterT>
  ParameterT getOrDeclareParameter(const std::string & name, const ParameterT & default_value)
  {
    if (this->has_parameter(name)) {
      ParameterT value = default_value;
      this->get_parameter(name, value);
      return value;
    }

    return this->declare_parameter<ParameterT>(name, default_value);
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    gm::Pointcloud::Ptr input_cloud(new gm::Pointcloud());
    pcl::fromROSMsg(*msg, *input_cloud);

    if (input_cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received an empty PointCloud2 message on '%s'.",
        pointcloud_topic_.c_str());
      return;
    }

    try {
      grid_map_pcl_loader_.setInputCloud(input_cloud);
      auto node = std::static_pointer_cast<rclcpp::Node>(this->shared_from_this());
      gm::processPointcloud(&grid_map_pcl_loader_, node);

      auto grid_map = grid_map_pcl_loader_.getGridMap();
      const std::string output_frame = map_frame_.empty() ? msg->header.frame_id : map_frame_;
      grid_map.setFrameId(output_frame);
      grid_map.setBasicLayers({map_layer_name_});

      auto grid_map_message = grid_map::GridMapRosConverter::toMessage(grid_map);
      grid_map_message->header.stamp = msg->header.stamp;
      grid_map_publisher_->publish(std::move(grid_map_message));
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to convert PointCloud2 to GridMap: %s",
        error.what());
    }
  }

  grid_map::GridMapPclLoader grid_map_pcl_loader_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  std::string pointcloud_topic_;
  std::string grid_map_topic_;
  std::string map_frame_;
  std::string map_layer_name_;
  std::string pcl_config_file_;
  int queue_size_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridMapPclPointcloudToGridMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}