#include <ros/ros.h>
#include <signal.h>

#include "euclidean_clustering/euclidean.h"

void sigint_handler(int signal)
{
  ros::shutdown();
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "euclidean_cluster_node", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigint_handler);
  ros::NodeHandle n;
  euclidean_clustering::EuclideanClustering::EuclideanClusterNode node(n);
  try
  {
    while(ros::ok())
    {
      ros::spin();
    }
  }
  catch(...)
  {
    ROS_ERROR("Caught some error of some sort...");
  }
}
