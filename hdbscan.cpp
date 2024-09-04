# HDBSCAN
#include "dbscan.h"

int DBSCAN::run()
{
    int clusterID = 1;
    vector<Point>::iterator iter;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( iter->clusterID == UNCLASSIFIED )
        {
            if ( expandCluster(*iter, clusterID) != FAILURE )
            {
                clusterID += 1;
            }
        }
    }

    return 0;
}

int DBSCAN::expandCluster(Point point, int clusterID)
{    
    vector<int> clusterSeeds = calculateCluster(point);
    //noize
    if ( clusterSeeds.size() < m_minPoints )
    {
        point.clusterID = NOISE;
        return FAILURE;
    }
    else
    {
        int index = 0, indexCorePoint = 0;
        vector<int>::iterator iterSeeds;
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            m_points.at(*iterSeeds).clusterID = clusterID;
            if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z )
            {
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

        for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
        {
            vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

            if ( clusterNeighors.size() >= m_minPoints )
            {
                vector<int>::iterator iterNeighors;
                for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                {
                    if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                    {
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED )
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points.at(*iterNeighors).clusterID = clusterID;
                    }
                }
            }
        }

        return SUCCESS;
    }
}

vector<int> DBSCAN::calculateCluster(Point point)
{
    int index = 0;
    vector<Point>::iterator iter;
    vector<int> clusterIndex;
    for( iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( calculateDistance(point, *iter) <= m_epsilon )
        {
            clusterIndex.push_back(index);
        }
        index++;
    }
    return clusterIndex;
}

inline double DBSCAN::calculateDistance(const Point& pointCore, const Point& pointTarget )
{
    return pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2);
}


// #include "pointcloud_clustering/cluster_node.hpp"

// PointCloudClusterNode::PointCloudClusterNode() : Node("pointcloud_cluster_node")
// {
//   subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//     "input_pointcloud", 10, std::bind(&PointCloudClusterNode::pointCloudCallback, this, std::placeholders::_1));

//   publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_pointcloud", 10);
// }

// void PointCloudClusterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//   pcl::fromROSMsg(*msg, *cloud);

//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//   tree->setInputCloud(cloud);

//   pcl::DBSCAN<pcl::PointXYZ> dbscan;
//   dbscan.setInputCloud(cloud);
//   dbscan.setSearchMethod(tree);
//   dbscan.setEps(0.02);  // Set epsilon value
//   dbscan.setMinPts(10);  // Set minimum points
//   dbscan.extract(clusters_);

//   RCLCPP_INFO(this->get_logger(), "Number of clusters found: %zu", clusters_.size());

//   // 各クラスタに色を付けてクラスタリング結果を1つの点群にまとめる
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//   int color_step = 255 / std::max(1, static_cast<int>(clusters_.size()));

//   for (size_t i = 0; i < clusters_.size(); ++i)
//   {
//     uint8_t r = (i * color_step) % 256;
//     uint8_t g = (i * color_step * 2) % 256;
//     uint8_t b = (i * color_step * 3) % 256;

//     for (const auto& idx : clusters_[i].indices)
//     {
//       pcl::PointXYZRGB point;
//       point.x = cloud->points[idx].x;
//       point.y = cloud->points[idx].y;
//       point.z = cloud->points[idx].z;
//       point.r = r;
//       point.g = g;
//       point.b = b;
//       clustered_cloud->points.push_back(point);
//     }
//   }

//   clustered_cloud->width = clustered_cloud->points.size();
//   clustered_cloud->height = 1;
//   clustered_cloud->is_dense = true;

//   // クラスタリング結果をパブリッシュする
//   publishClusteredPointCloud(clustered_cloud);
// }

// void PointCloudClusterNode::publishClusteredPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clustered_cloud)
// {
//   sensor_msgs::msg::PointCloud2 output_msg;
//   pcl::toROSMsg(*clustered_cloud, output_msg);
//   output_msg.header.frame_id = "map";  // 適切なフレームIDを設定
//   output_msg.header.stamp = this->now();

//   publisher_->publish(output_msg);
//   RCLCPP_INFO(this->get_logger(), "Published clustered point cloud with %zu points", clustered_cloud->points.size());
// }

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloudClusterNode>());
//   rclcpp::shutdown();
//   return 0;
// }

