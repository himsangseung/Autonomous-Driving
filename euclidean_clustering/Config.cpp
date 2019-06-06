#include "euclidean_clustering/Config.h"
//#include "euclidean_clustering/euclidean.h"
namespace euclidean_clustering
{
namespace EuclideanClustering
{

namespace Config
{

Config::Config(const ros::NodeHandle & n)
{
  int cluster_min_size, cluster_max_size;
  n.param<int>("cluster_min_size", cluster_min_size, 100);
  _config.cluster_min_size = cluster_min_size > 0 ? cluster_min_size : 100;
  n.param<int>("cluster_max_size", cluster_max_size, 1000);
  _config.cluster_max_size = cluster_max_size > 0 ? cluster_max_size : 1000;
  n.param<double>("cluster_tolerance", _config.cluster_tolerance, 0.15);
  n.param<bool>("downsample", _config.downsample, true);
  n.param<double>("downsample_leaf_size", _config.downsample_leaf_size, 0.5);
  n.param<bool>("bound_obstacles", _config.bound_obstacles, false);
  n.param<bool>("clip_height", _config.clip_height, true);
  n.param<double>("clip_height_min", _config.clip_height_min, -10.0);
  n.param<double>("clip_height_max", _config.clip_height_max, 0.0);
  n.param<bool>("clip_close", _config.clip_close, false);
  n.param<double>("clip_close_xmin", _config.clip_close_xmin, -2.0);
  n.param<double>("clip_close_xmax", _config.clip_close_xmax, 0.75);
  n.param<double>("clip_close_ymin", _config.clip_close_ymin, -0.75);
  n.param<double>("clip_close_ymax", _config.clip_close_ymin, 0.75);

  _d_reconfig.setCallback(boost::bind(&Config::dynamic_reconfigure_callback, this, _1, _2));

};

ConfigOptions Config::get_config()
{
  ConfigOptions return_config;
  _config_mutex.lock();
  return_config = _config;
  _config_mutex.unlock();
  return return_config;
}

void Config::dynamic_reconfigure_callback(euclidean_clustering::EuclideanClusterDynamicConfig & config, uint32_t level)
{
  _config_mutex.lock();
  _config.cluster_min_size = config.cluster_min_size;
  _config.cluster_max_size = config.cluster_max_size;
  _config.cluster_tolerance = config.cluster_tolerance;
  _config.downsample = config.downsample;
  _config.downsample_leaf_size = config.downsample_leaf_size;
  _config.bound_obstacles = config.bound_obstacles;
  _config.clip_height = config.clip_height;
  _config.clip_height_min = config.clip_height_min;
  _config.clip_height_max = config.clip_height_max;
  _config.clip_close = config.clip_close;
  _config.clip_close_xmin = config.clip_close_xmin;
  _config.clip_close_xmax = config.clip_close_xmax;
  _config.clip_close_ymin = config.clip_close_ymin;
  _config.clip_close_ymax = config.clip_close_ymax;
  _config_mutex.unlock();
}

} // namespace Config

} // namespace EuclideanClustering

} //namespace euclidean_clustering
