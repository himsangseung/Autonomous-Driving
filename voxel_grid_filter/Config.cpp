#include "voxel_grid_filter/Config.h"
#include <iostream>

namespace pcl_nodelets
{
namespace voxel_grid_filter
{
namespace Config
{
// min voxel size for configuration
Config::Config(const ros::NodeHandle & n)
{
  double grid_size = 0.15;
  n.param<double>("grid_size", _config.grid_size, 0.15);

  _valid = true;
  _d_reconfig.setCallback(boost::bind(&Config::dynamic_reconfigure_callback, this, _1, _2));
}

ConfigOptions Config::get_config()
{
  ConfigOptions return_config;
  _config_mutex.lock();
  return_config = _config;
  _config_mutex.unlock();
  return return_config;
}

void Config::dynamic_reconfigure_callback(pcl_nodelets::VoxelGridFilterDynamicConfig & config, uint32_t level)
//void dynamic_reconfigure_callback(pcl_nodelets::VoxelGridFilterDynamic & config, uint32_t level)
{
  //dont' think we need timping slop?
  _config_mutex.lock();
  _config.grid_size = config.grid_size;
  _config.pointcld_topic = config.pointcld_topic;
  _config_mutex.unlock();
}

} // namespace Config
} // namespace voxel_grid_filter
} // namespace pcl_nodelets
