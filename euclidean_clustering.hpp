#ifndef EUCLIDEAN_CLUSTERING_NODE_HPP_
#define EUCLIDEAN_CLUSTERING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

class EuclideanClusteringNode : public rclcpp::Node
{
public:
  EuclideanClusteringNode();
  
private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

#endif  // EUCLIDEAN_CLUSTERING_NODE_HPP_
