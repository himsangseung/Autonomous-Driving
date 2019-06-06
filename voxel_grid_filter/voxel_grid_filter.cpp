#include <voxel_grid_filter/VoxelGridFilter.h>
#include <std_msgs/String.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <typeinfo>

namespace pcl_nodelets
{
namespace voxel_grid_filter
{
VoxelGridFilter::VoxelGridFilter(ros::NodeHandle & n, ros::NodeHandle & pn) ://two node handlers....
  _config(pn)
{
  //std::string topic = _config.topic;
  ROS_INFO("Starting in constructor");
  ROS_INFO("config_valid: %s", _config.valid() ? "true" : "false");
  //Check config to see if it's valid
  if (_config.valid()){
    Config::ConfigOptions conf = _config.get_config();

    pcl_sub = n.subscribe<sensor_msgs::PointCloud2>(conf.pointcld_topic,1, &VoxelGridFilter::cb_callback, this);
    pcl_pub =  n.advertise<sensor_msgs::PointCloud2>("/voxel_filtered", 5);
  }
}// constuctor

void VoxelGridFilter::cb_callback(sm_pcl2_type c)
{

    ROS_INFO("calls callback");

    ROS_INFO("gets here");
    ROS_INFO("Input size: %d", c->width * c->height);
    Config::ConfigOptions conf = _config.get_config();
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2() );
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2() );
    pcl_conversions::toPCL(*c, *pcl_pc2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    ROS_INFO("conversion to pcl");
    voxel_grid.setInputCloud (pcl_pc2);
    voxel_grid.setLeafSize( conf.grid_size, conf.grid_size, conf.grid_size); //make it a variable but now at 1 cm
    voxel_grid.filter(*cloud_filtered);


    ROS_INFO("vgf completed");
    //sensor_msgs::PointCloud2 output;
    //pcl::toROSMsg(&(*cloud_filtered), &output);

    pcl_pub.publish(*cloud_filtered);// publish cloudfiltered
    /*
    pcl::PointCloud<pcl::PointXYZ> pcl_xyz;
    pcl::fromPCLPointCloud2( *cloud_filtered, pcl_xyz);

    for ( size_t i = 0; i < pcl_xyz.size() ; ++i){
      std::cout<<pcl_xyz.points[i].z<<std::endl;
    }
    */


    ROS_INFO("gets here3");
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered, output);
    ROS_INFO("Output size: %d", output.width * output.height);
    pcl_pub.publish(output);// publish cloudfiltered
} // call back
} // namespace voxel_grid_filter

} // namespace pcl_nodelets
