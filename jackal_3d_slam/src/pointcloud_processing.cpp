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
    // Initializes variables for the parameters, rate, publisher, subscriber, and timer
    declare_parameter("x_filter_min", -0.5);
    declare_parameter("x_filter_max", 0.0);
    declare_parameter("z_filter_min", -0.26);
    declare_parameter("z_filter_max", 0.13);
    declare_parameter("search_radius", 0.8);
    declare_parameter("num_neighbors", 2);
    declare_parameter("voxel_leaf_size", 0.01);

    x_filter_min_ = get_parameter("x_filter_min").get_parameter_value().get<double>();
    x_filter_max_ = get_parameter("x_filter_max").get_parameter_value().get<double>();
    z_filter_min_ = get_parameter("z_filter_min").get_parameter_value().get<double>();
    z_filter_max_ = get_parameter("z_filter_max").get_parameter_value().get<double>();
    search_radius_ = get_parameter("search_radius").get_parameter_value().get<double>();
    num_neighbors_ = get_parameter("num_neighbors").get_parameter_value().get<int>();
    voxel_leaf_size_ = float(get_parameter("voxel_leaf_size").get_parameter_value().get<double>());

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

  /// \brief Filters the pointcloud data using PassThrough, RadiusOutlierRemoval, and VoxelGrid
  /// filters
  ///
  /// \param cloud_in - the pointcloud data to filter
  /// \return the filtered pointcloud data
  pcl::PointCloud<pcl::PointXYZI>::Ptr filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_in)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out{new pcl::PointCloud<pcl::PointXYZI>()};
    pcl::PassThrough<pcl::PointXYZI> pt1, pt2;
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad_outlier;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;

    // Filter the data along z
    pt1.setInputCloud(cloud_in);
    pt1.setFilterFieldName("z");
    pt1.setFilterLimits(z_filter_min_, z_filter_max_);
    pt1.filter(*cloud_out);

    // Filter the data along x
    pt2.setInputCloud(cloud_out);
    pt2.setFilterFieldName("x");
    pt2.setFilterLimits(x_filter_min_, x_filter_max_);
    pt2.setNegative(true);
    pt2.filter(*cloud_out);

    // Remove outliers
    rad_outlier.setInputCloud(cloud_out);
    rad_outlier.setRadiusSearch(search_radius_);
    rad_outlier.setMinNeighborsInRadius(num_neighbors_);
    rad_outlier.filter(*cloud_out);

    // Downsample the data
    voxel_grid.setInputCloud(cloud_out);
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
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

  double x_filter_min_, x_filter_max_, z_filter_min_, z_filter_max_, search_radius_;
  int num_neighbors_, rate_;
  float voxel_leaf_size_;
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
