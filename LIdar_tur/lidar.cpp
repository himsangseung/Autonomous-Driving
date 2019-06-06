#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// Note: global variables like this are bad practice, but is done here for simplicity.
// A more elegant solution would be to write your ROS node as a class, and have the
// publishers and subscribers be private variables inside said class.
ros::Publisher pcl_pub;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr & input_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*input_msg, *input_pcl);
  // This code runs fast, but can be a bit difficult to read if you aren't
  // familiar with libPCL. An loop that performs the same operation but is slower
  // has been provided for your reference
  /*
  for(auto it = input_pcl->begin(); it<input_pcl->end(); it++)
  {
    if(it->x < 0 or it->x > 100)
    {
      input_pcl->erase(it);
      --it;
    }
  }
  */
  pcl::PassThrough<pcl::PointXYZI> filter;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  filter.setInputCloud(input_pcl);
  filter.setFilterFieldName("x");
  filter.setFilterLimits(0.0,100.0);
  filter.filter(*filtered);
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*filtered, output_msg);

  output_msg.header = input_msg->header;
  pcl_pub.publish(output_msg);
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "lab1_node");
  ros::NodeHandle nh;
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("points_modified", 2, true);
  ros::Subscriber pcl_sub = nh.subscribe("points_raw", 2, pointcloud_callback);
  ros::spin();
  return 0;
}
