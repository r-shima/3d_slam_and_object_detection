/// \file
/// \brief This node removes and downsamples noisy pointcloud data
///
/// PUBLISHES:
///     /filtered_velodyne_points (sensor_msgs::msg::PointCloud2): filtered pointcloud data
/// SUBSCRIBES:
///     /velodyne_points (sensor_msgs::msg::PointCloud2): Pointcloud data from the Velodyne

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

/// \brief Performs pointcloud filtering
class PointcloudProcessing : public rclcpp::Node
{
public:
  PointcloudProcessing()
  : Node("pointcloud_processing")
  {
    // Initializes variables for the rate, publisher, subscriber, and timer
    rate_ = 200;
    filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_velodyne_points", 10);
    velodyne_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(
        &PointcloudProcessing::velodyne_points_callback, this,
        std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate_),
      std::bind(&PointcloudProcessing::timer_callback, this));
  }

private:
  /// \brief Callback function for the subscriber that subscribes to sensor_msgs/msg/PointCloud2.
  /// Filters the pointcloud data.
  ///
  /// \param msg - PointCloud2 object
  /// \returns none
  void velodyne_points_callback(const sensor_msgs::msg::PointCloud2 & msg)
  {
    // Convert ROS2 PointCloud2 to PointCloud<PointXYZI>
    pcl::fromROSMsg(msg, *pcl_cloud_);

    // // Convert PCL PointCloud2 to PointCloud<PointXYZI>
    // pcl::fromPCLPointCloud2(*pcl_cloud2_, *pcl_cloud_);
    
    pcl_cloud_ = filter(pcl_cloud_);

    // // Convert PointCloud<PointXYZI> to PCL PointCloud2
    // pcl::toPCLPointCloud2(*pcl_cloud_, *pcl_cloud2_conv_);

    // Convert PointCloud<PointXYZI> to ROS2 PointCloud2
    pcl::toROSMsg(*pcl_cloud_, cloud_);
  }

  /// \brief Filter the data using PassThrough, RadiusOutlierRemoval, and VoxelGrid
  ///
  /// \param cloud_in - the pointcloud data to filter
  /// \return the filtered pointcloud data
  pcl::PointCloud<pcl::PointXYZI>::Ptr filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_in)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out{new pcl::PointCloud<pcl::PointXYZI>()};
    pcl::PassThrough<pcl::PointXYZI> z_data, x_data;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad_outlier;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

    // Filter in z direction
    z_data.setInputCloud(cloud_in);
    z_data.setFilterFieldName("z");
    z_data.setFilterLimits(-0.26, 0.13);
    z_data.filter(*cloud_out);

    // Filter in x direction
    x_data.setInputCloud(cloud_out);
    x_data.setFilterFieldName("x");
    x_data.setFilterLimits(-0.5, 0.0);
    x_data.setNegative(true);
    x_data.filter(*cloud_out);

    // Remove outliers
    rad_outlier.setInputCloud(cloud_out);
    rad_outlier.setRadiusSearch(0.8);
    rad_outlier.setMinNeighborsInRadius(2);
    rad_outlier.filter(*cloud_out);

    // Downsample the data
    voxel_grid.setInputCloud(cloud_out);
    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_grid.filter(*cloud_out);

    return cloud_out;
  }

  /// \brief Callback function for the timer. Publishes filtered pointcloud data.
  ///
  /// \param none
  /// \returns none
  void timer_callback()
  {
    filtered_points_pub_->publish(cloud_);
  }

  // Declare private variables
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int rate_;
  // pcl::PCLPointCloud2::Ptr pcl_cloud2_{new pcl::PCLPointCloud2()};
  // pcl::PCLPointCloud2::Ptr pcl_cloud2_conv_{new pcl::PCLPointCloud2()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_{new pcl::PointCloud<pcl::PointXYZI>()};
  sensor_msgs::msg::PointCloud2 cloud_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudProcessing>());
  rclcpp::shutdown();
  return 0;
}
