// ROS
#include <ros/ros.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/convex_hull.h>

// C++11
#include <string>

// Local
#include "euclidean_clustering/euclidean.h"

namespace euclidean_clustering
{
namespace EuclideanClustering
{


EuclideanClusterNode::EuclideanClusterNode(ros::NodeHandle & n) : _config(n),
    //pcl_sub(n.subscribe("velodyne_points", 1, &EuclideanClusterNode::cloud_cb, this)),
    pcl_sub(n.subscribe("/vlp16_2/velodyne_points", 1, &EuclideanClusterNode::cloud_cb, this)),
    pc_pub(n.advertise<sensor_msgs::PointCloud2>("pc_follower_cloud", 1)),
    vis_pub(n.advertise<visualization_msgs::Marker>("centroid_marker", 0)),
    cmd_pub(n.advertise<geometry_msgs::Twist>("cmd_vel", 0)),
    poly_pub(n.advertise<jsk_recognition_msgs::PolygonArray>("obstacle_polygons", 1))
{


}

EuclideanClusterNode::~EuclideanClusterNode()
{

}

void EuclideanClusterNode::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud_input);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clipped(nullptr), cloud_downsampled(nullptr), cloud_filtered(nullptr);
  Config::ConfigOptions conf = _config.get_config();
  //std::cout << "HERE!" << std::endl;


  // Clip close
  if(conf.clip_close)
  {
    Eigen::Vector4f min_pt, max_pt;
    min_pt[0] = conf.clip_close_xmin;
    min_pt[1] = conf.clip_close_ymin;
    min_pt[2] = -100.0;
    max_pt[0] = conf.clip_close_xmax;
    max_pt[1] = conf.clip_close_ymax;
    max_pt[2] = 100.0;
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloud_input);
    cloud_clipped.reset(new pcl::PointCloud<pcl::PointXYZ>());
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.setNegative(true);
    crop.filter(*cloud_clipped);
  }
  else
  {
    cloud_clipped = cloud_input;
  }

  //-----------------------------------------------------------------------------
  //--------- DOWNSAMPLE
  //--------------------------------------------------------
  if(conf.downsample)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_clipped);
    cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>());
    vg.setLeafSize (conf.downsample_leaf_size,
                    conf.downsample_leaf_size,
                    conf.downsample_leaf_size);

    vg.filter(*cloud_downsampled);
  }
  else
  {
    cloud_downsampled = cloud_clipped;
  }

  //-----------------------------------------------------------------------------
  //--------- REMOVE GROUND PLANE
  //-----------------------------------------------

  // Assume the ground plane is below the lidar
  // Only pass through points whose Z coordinate falls between 0 and -10
  if(conf.clip_height)
  {
    cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_downsampled);
    pass.setFilterFieldName("z");
    if(conf.clip_height_min > conf.clip_height_max)
    {
      pass.setFilterLimits(conf.clip_height_max, conf.clip_height_min);
    }
    else
    {
      pass.setFilterLimits(conf.clip_height_min, conf.clip_height_max);
    }
    pass.filter(*cloud_filtered);
  }
  else
  {
    cloud_filtered = cloud_downsampled;
  }

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new
  pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new
  pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.5);

  int nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_INFO_STREAM("Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points.");

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }


  //-----------------------------------------------------------------------------
  //--------- CLUSTER
  //-----------------------------------------------------------
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  // float size = cloud_filtered->points.size();

  ec.setClusterTolerance(conf.cluster_tolerance);  // meters

  ec.setMinClusterSize(conf.cluster_min_size);

  ec.setMaxClusterSize(conf.cluster_max_size);

  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  // ROS_INFO_STREAM(cluster_indices.size() << " clusters extracted");

  int cluster_index = 0;
  // iterate through clusters
  int id = 0;
  jsk_recognition_msgs::PolygonArray cluster_polygons;
  for (pcl::PointIndices & cluster : cluster_indices)
//const pcl::PointIndices & cluster : cluster_indices)
  {
    std::vector<int> & cluster_vector = cluster.indices;
    int row, column;
    Eigen::Vector4f centroid;

    // compute centroid of cluster
    pcl::compute3DCentroid(*cloud_filtered, cluster, centroid);


    // Attempt to get 2D polygon obstacle bounds
    if(conf.bound_obstacles)
    {
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(
         new pcl::PointCloud<pcl::PointXYZ>);
       pcl::ProjectInliers<pcl::PointXYZ> proj;
       proj.setModelType(pcl::SACMODEL_PLANE);
       proj.setInputCloud(cloud_filtered);
       // libPCL doesn't know how to convert from pcl::PointIndices
       // to pcl::IndicesPtr
       pcl::IndicesPtr cluster_indices_ptr = boost::make_shared<std::vector<int>  >(cluster.indices);
       proj.setIndices(cluster_indices_ptr);
       proj.setModelCoefficients(coefficients);
       proj.filter(*cloud_projected);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(
	  new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cloud_projected);
        chull.setDimension(2);
        chull.reconstruct(*cloud_hull);
	geometry_msgs::PolygonStamped new_obstacle_poly;
        new_obstacle_poly.header.frame_id = input->header.frame_id;
        new_obstacle_poly.header.stamp = input->header.stamp;
        for(auto & i : *cloud_hull)
        {
          geometry_msgs::Point32 new_pt;
          new_pt.x = i.x;
          new_pt.y = i.y;
          new_pt.z = i.z;
          new_obstacle_poly.polygon.points.push_back(new_pt);
        }
        cluster_polygons.polygons.push_back(new_obstacle_poly);
        cluster_polygons.labels.push_back(0);
        cluster_polygons.likelihood.push_back(.9);
        std::cerr << std::endl;
	//end bounding obstacles
      }
    // if (centroid[0] > 0.50) {
    //  if (centroid[0] < 3.5) {
    //   if (centroid[1] < 1.0) {
    //     if (centroid[1] > -1.0) {

           // geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
           // std::cout << "NOW!" << std::endl;
          geometry_msgs::Twist cmd;

            //std::cout << "linear_vel: "<< (centroid[0] - goal_x) * x_scale << std::endl;
            //std::cout << "ang_vel: "<< std::abs(centroid[1]) * y_scale << std::endl;


          float goal_x = 0.5;
          float x_scale = 0.3;
          float y_scale = 0.5;

            if (centroid[0] < 1.0){
               cmd.linear.x = -( (centroid[0] - 0.5) * 0.2  );
               cmd.angular.z = centroid[1] * y_scale;
            }
            else{
                cmd.linear.x = (centroid[0] - goal_x) * x_scale;
                cmd.angular.z = centroid[1] * y_scale;
            }

          cmd_pub.publish(cmd);


          visualization_msgs::Marker marker;
          marker.header.frame_id = input->header.frame_id;  // Copy frame ID from input
          marker.header.stamp = input->header.stamp;        // Time as well
          marker.ns = "my_namespace";
          marker.id = id;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          // marker.text = std::to_string(id);
          // marker.text = "KYLE";
          marker.lifetime = ros::Duration(1);
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 0.0;
          marker.scale.x = 1.0;
          marker.scale.y = 1.0;
          marker.scale.z = 1.0;
          marker.color.a = 0.35;  // Don't forget to set the alpha!

          marker.pose.position.x = centroid[0];
          marker.pose.position.y = centroid[1];
          marker.pose.position.z = centroid[2];
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;

          vis_pub.publish(marker);
          id += 1;


          std::cout << "CENTROID AT: " << std::endl;
          std::cout << "X: " << centroid[0] << std::endl;
          std::cout << "Y: " << centroid[1] << std::endl;
          std::cout << "Z: " << centroid[2] << std::endl;

    //     }  // end of if (looking for forward)
    //    }
    //   }
    // }

        // else {
        //   std::cout
        //       << "Not enough points detected, or out of bounds, stopping the "
        //          "robot \n"
        //       << std::endl;
        //
        //     geometry_msgs::Twist stop;
        //     cmd_pub.publish(stop);
        //
        // }  // end of else

      }  // end for loop
      if(conf.bound_obstacles)
      {
        cluster_polygons.header.frame_id = input->header.frame_id;
        cluster_polygons.header.stamp = input->header.stamp;
        poly_pub.publish(cluster_polygons);
      }
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*cloud_filtered, output);
      pc_pub.publish(output);



    }  // end of cb
} // namespace euclidean_clustering
} //namespace euclidean_clustering
