#include "rclcpp/rclcpp.hpp"
#include "point_clustering/euclidean_clustering_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<euclidean_clustering::EuclideanClustering>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}