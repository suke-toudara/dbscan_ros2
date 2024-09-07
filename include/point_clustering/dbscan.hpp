#ifndef POINTCLOUD_CLUSTERING_CLUSTER_NODE_HPP_
#define POINTCLOUD_CLUSTERING_CLUSTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/dbscan.h>
#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;
typedef struct Point_
{
    float x, y, z;  // X, Y, Z position
    int clusterID;  // clustered ID
}Point;

class DBSCAN : public rclcpp::Node
{
public:
  PointCloudClusterNode();
    DBSCAN(unsigned int minPts, float eps, vector<Point> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN(){}
private:    
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
    int run();
    vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(const Point& pointCore, const Point& pointTarget);
    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    vector<Point> m_points;
    //function
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishClusteredPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& clustered_cloud);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::vector<pcl::PointIndices> clusters_;
};

#endif // DBSCAN_H
