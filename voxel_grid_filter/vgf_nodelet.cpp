#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "voxel_grid_filter/VoxelGridFilter.h"
#include "voxel_grid_filter/Config.h"

namespace pcl_nodelets
{
  class vgf_nodelet : public nodelet::Nodelet
  {
  public:
    vgf_nodelet() : conv_(nullptr) {}
    ~vgf_nodelet() {}
  private:
    virtual void onInit();
    //boost::shared_ptr<pc_fusion::VoxelGridFilter> conv_;
    boost::shared_ptr<voxel_grid_filter::VoxelGridFilter> conv_;
  };

  /** @brief Nodelet initialization. */
  void vgf_nodelet::onInit()
  {
    vgf_nodelet::conv_.reset(new voxel_grid_filter::VoxelGridFilter(getNodeHandle(), getPrivateNodeHandle()));
    //vgf_nodelet::conv_.reset(new pc_fusion::VoxelGridFilter(getNodeHandle(), getPrivateNodeHandle()));
    //ROS_INFO ("IN NODELET CPP, ONINIT METHOD");
  }

} // namespace velodyne_multi_fusion

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
//PLUGINLIB_EXPORT_CLASS(votex_grid_filter::vgf_nodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(pcl_nodelets::vgf_nodelet, nodelet::Nodelet)
