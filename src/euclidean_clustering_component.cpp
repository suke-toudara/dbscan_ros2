#include "point_clustering/euclidean_clustering_component.hpp"

namespace euclidean_clustering
{
EuclideanClustering::EuclideanClustering(const rclcpp::NodeOptions & options) : Node("pclsub", options)
{
  declare_parameter("sub_pc_topic","/sensing/velodyne_lower/velodyne_points");
  declare_parameter("sac_segmentation",true);
  declare_parameter("voxel_size",0.02);
  declare_parameter("cluster_tolerance",0.2); // 2cm
  declare_parameter("min_cluster_size",200);
  declare_parameter("max_cluster_size",1000);
  get_parameter("sub_pc_topic",sub_pc_topic_);
  get_parameter("sac_segmentation",sac_segmentation_);
  get_parameter("voxel_size",voxel_size_);
  get_parameter("min_cluster_size",min_cluster_size_);
  get_parameter("cluster_tolerance",cluster_tolerance_);
  get_parameter("max_cluster_size",max_cluster_size_);
  get_parameter("max_cluster_size",max_cluster_size_);
  
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).best_effort();
  pc_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_pc_topic_,qos,[this](sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        pc_callback(cloud_msg);
        });
  pc_cluster_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 10);
}


void EuclideanClustering::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Voxel 
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  voxelGrid.setInputCloud(cloud);
  voxelGrid.setLeafSize(voxel_size_,voxel_size_,voxel_size_); // set the leaf size (x, y, z)   // LeafSizeを細かくしすぎると、エラーとなり、止まる
  voxelGrid.filter(*cloud_filtered);

  // Create the KdTree object 
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  // SAC Segmentation
  if (sac_segmentation_){    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
    // Create the segmentation object  
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
    double threshould = 0.01;
    // Optional  
    seg.setOptimizeCoefficients (true);  
    // Mandatory  
    seg.setModelType (pcl::SACMODEL_PLANE);  
    seg.setMethodType (pcl::SAC_RANSAC);  
    seg.setDistanceThreshold (threshould);  
    seg.setInputCloud (cloud_filtered);  
    seg.segment (*inliers, *coefficients);  
  }
  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
  // specify euclidean cluster parameters
  ece.setClusterTolerance (cluster_tolerance_); 
  ece.setMinClusterSize (min_cluster_size_);
  ece.setMaxClusterSize (max_cluster_size_);
  ece.setSearchMethod (tree);
  ece.setInputCloud (cloud_filtered);
  ece.extract (cluster_indices);

  int j = 0;  
  float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);  
  pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  
    {  
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {  
        cloud_cluster->points[*pit].r = colors[j%6][0];  
        cloud_cluster->points[*pit].g = colors[j%6][1];  
        cloud_cluster->points[*pit].b = colors[j%6][2];  
      }  
      j++;  
    }  

  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_cluster, sensor_msg);
  pc_cluster_pub_->publish(sensor_msg);
}
} // euclidean_clustering

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(euclidean_clustering::EuclideanClustering)
