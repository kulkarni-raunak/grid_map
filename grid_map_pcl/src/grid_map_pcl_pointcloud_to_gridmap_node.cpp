/*
 * grid_map_pcl_pointcloud_to_gridmap_node.cpp
 */

#include <pcl_conversions/pcl_conversions.h>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <cmath>

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
    max_abs_xy_m_ = getOrDeclareParameter<double>("max_abs_xy_m", 150.0);
    max_abs_z_m_ = getOrDeclareParameter<double>("max_abs_z_m", 20.0);

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

    gm::Pointcloud::Ptr sanitized_cloud = sanitizePointcloud(input_cloud);
    if (sanitized_cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "All incoming points were filtered out as invalid/out-of-range. Check topic '%s' and bounds (max_abs_xy_m=%.1f, max_abs_z_m=%.1f).",
        pointcloud_topic_.c_str(), max_abs_xy_m_, max_abs_z_m_);
      return;
    }

    try {
      grid_map_pcl_loader_.setInputCloud(sanitized_cloud);
      auto node = std::static_pointer_cast<rclcpp::Node>(this->shared_from_this());
      gm::processPointcloud(&grid_map_pcl_loader_, node);

      auto grid_map = grid_map_pcl_loader_.getGridMap();
      const std::string output_frame = map_frame_.empty() ? msg->header.frame_id : map_frame_;
      grid_map.setFrameId(output_frame);
      grid_map.setBasicLayers({map_layer_name_});

      auto grid_map_message = grid_map::GridMapRosConverter::toMessage(grid_map);
      grid_map_message->header.stamp = msg->header.stamp;
      grid_map_publisher_->publish(std::move(grid_map_message));
    } catch (const std::bad_alloc &) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to convert PointCloud2 to GridMap: std::bad_alloc. Likely cloud extent is too large; tighten max_abs_xy_m/max_abs_z_m or increase voxel size/resolution.");
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to convert PointCloud2 to GridMap: %s",
        error.what());
    }
  }

  gm::Pointcloud::Ptr sanitizePointcloud(const gm::Pointcloud::ConstPtr & input_cloud) const
  {
    gm::Pointcloud::Ptr sanitized_cloud(new gm::Pointcloud());
    sanitized_cloud->points.reserve(input_cloud->points.size());

    for (const auto & point : input_cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      if (std::fabs(point.x) > max_abs_xy_m_ ||
        std::fabs(point.y) > max_abs_xy_m_ ||
        std::fabs(point.z) > max_abs_z_m_)
      {
        continue;
      }

      sanitized_cloud->points.push_back(point);
    }

    sanitized_cloud->width = sanitized_cloud->points.size();
    sanitized_cloud->height = 1;
    sanitized_cloud->is_dense = true;
    return sanitized_cloud;
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
  double max_abs_xy_m_;
  double max_abs_z_m_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridMapPclPointcloudToGridMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}