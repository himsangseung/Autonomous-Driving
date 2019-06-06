#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "euclidean_clustering/euclidean.h"
#include "euclidean_clustering/Config.h"

namespace euclidean_clustering
{
  class ec_nodelet : public nodelet::Nodelet
  {
  public:
    ec_nodelet() : conv_(nullptr) {}
    ~ec_nodelet() {}
  private:
    virtual void onInit();
    //boost::shared_ptr<pc_fusion::VoxelGridFilter> conv_;
    boost::shared_ptr<EuclideanClustering::EuclideanClusterNode> conv_;
  };

  /** @brief Nodelet initialization. */
  void ec_nodelet::onInit()
  {
    //ec_nodelet::conv_.reset(new voxel_grid_filter::VoxelGridFilter(getNodeHandle(), getPrivateNodeHandle()));
    //ec_nodelet::conv_.reset(new EuclideanClustering::EuclideanClusterNode(getNodeHandle(), getPrivateNodeHandle()));
ec_nodelet::conv_.reset(new EuclideanClustering::EuclideanClusterNode(getNodeHandle()));
    //ROS_INFO ("IN NODELET CPP, ONINIT METHOD");
  }

} // namespace euclidean_clustering

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
//PLUGINLIB_EXPORT_CLASS(votex_grid_filter::ec_nodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(euclidean_clustering::ec_nodelet, nodelet::Nodelet)
