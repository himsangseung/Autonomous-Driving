#include <ros/ros.h>
#include <voxel_grid_filter/VoxelGridFilter.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_voxel_grid_filter_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pcl_nodelets::voxel_grid_filter::VoxelGridFilter node(n, pn);
  ros::spin();
}
