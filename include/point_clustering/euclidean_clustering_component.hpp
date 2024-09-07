#ifndef EUCLIDEAN_CLUSTERING_NODE_HPP_
#define EUCLIDEAN_CLUSTERING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace euclidean_clustering
{
class EuclideanClustering : public rclcpp::Node
{
  public:
    explicit EuclideanClustering(const rclcpp::NodeOptions & options);
  private:
    std::string sub_pc_topic_ ; 
    bool sac_segmentation_ ; 
    float voxel_size_ ;
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_cluster_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber_;
};
}
#endif // EUCLIDEAN_CLUSTERING_NODE_HPP_