cmake_minimum_required(VERSION 3.8)
project(point_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
#find_package(ament_cmake REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)

# find_package(PCL 1.1 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# install(DIRECTORY
#   #launch
#   #config
#   #DESTINATION share/${PROJECT_NAME}/
# )

ament_auto_find_build_dependencies()

ament_auto_add_library(euclidean_clustering src/euclidean_clustering_component.cpp)
    
rclcpp_components_register_nodes(euclidean_clustering
    "euclidean_clustering::EuclideanClustering")


ament_auto_add_executable(euclidean_clustering_node
  src/euclidean_clustering_component.cpp
  src/euclidean_clustering_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
