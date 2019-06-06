#include <pointcloud_fusion/PointCloudFusion.h>
#include <std_msgs/String.h>
#include <iostream>
namespace velodyne_multi_fusion
{
namespace pc_fusion
{
PointCloudFusion::PointCloudFusion(ros::NodeHandle & n, ros::NodeHandle & pn) ://two node handlers....
  _config(pn)
{
  ROS_INFO("Starting in constructor");
  ROS_INFO("config_valid: %s", _config.valid() ? "true" : "false");
  //Check config to see if it's valid
  if (_config.valid()){
    // Then populate subscribers
    Config::ConfigOptions conf = _config.get_config();
    _subscribers.resize(conf.pcl_topics.size());
    unsigned int subscriber_id = 0;
    for(const std::string & topic : conf.pcl_topics)
    {

      ROS_INFO("Subscribing to: %s", topic.c_str());

      std::shared_ptr<msg_sub> sub_n(new msg_sub(n,topic, 10));

      _subscribers[subscriber_id++] = sub_n;
      //ROS_INFO("In PCL Fusion Constructor.");


      //ROS_INFO( "new thing %i and %i", a ,(conf.pcl_topics.size()));

    }
    ROS_INFO( "new thing %lu and %lu", _subscribers.size() ,(conf.pcl_topics.size()));
    // Then make approx time synchronization object
    sm_pcl2_ptr null_pcl;
    switch(this->_subscribers.size())// typedef included in type_def.h
    {
      case(2):
      {
        ROS_INFO("CASE TWO");
        _sync2 = std::make_shared<Synchronizer<sync_pcl_2> >(sync_pcl_2(10), *_subscribers[0], *_subscribers[1]);
        _sync2->registerCallback(boost::bind(&PointCloudFusion::sync_pcl_callback, this, _1, _2, null_pcl, null_pcl, null_pcl, null_pcl, null_pcl, null_pcl));
        ROS_INFO("Callback registered");

        break;
      }
      case(3):
      {
        _sync3 = std::make_shared<Synchronizer<sync_pcl_3> >(sync_pcl_3(10), *_subscribers[0], *_subscribers[1],
                                                            *_subscribers[2]);
        _sync3->registerCallback(boost::bind(&PointCloudFusion::sync_pcl_callback, this, _1, _2, _3, null_pcl, null_pcl, null_pcl, null_pcl, null_pcl));
        break;
      }
      case(4):
      {
        _sync4 = std::make_shared<Synchronizer<sync_pcl_4> >(sync_pcl_4(10), *_subscribers[0], *_subscribers[1],
                                                            *_subscribers[2], *_subscribers[3]);
        _sync4->registerCallback(boost::bind(&PointCloudFusion::sync_pcl_callback, this, _1, _2, _3, _4, null_pcl, null_pcl, null_pcl, null_pcl));
        break;
      }
      case(5):
      {
        _sync5 = std::make_shared<Synchronizer<sync_pcl_5> >(sync_pcl_5(10), *_subscribers[0], *_subscribers[1],
                                                            *_subscribers[2], *_subscribers[3], *_subscribers[4]);
        _sync5->registerCallback(boost::bind(&PointCloudFusion::sync_pcl_callback, this, _1, _2, _3, _4, _5, null_pcl, null_pcl, null_pcl));
        break;
      }
      case(6):
      {
        _sync6 = std::make_shared<Synchronizer<sync_pcl_6> >(sync_pcl_6(10), *_subscribers[0], *_subscribers[1],
                                                            *_subscribers[2], *_subscribers[3], *_subscribers[4],
                                                            *_subscribers[5]);
        _sync6->registerCallback(boost::bind(&PointCloudFusion::sync_pcl_callback, this, _1, _2, _3, _4, _5, _6, null_pcl, null_pcl));
        break;
      }
      case(7):
      {
        _sync7 = std::make_shared<Synchronizer<sync_pcl_7> >(sync_pcl_7(10), *_subscribers[0], *_subscribers[1],
                                                            *_subscribers[2], *_subscribers[3], *_subscribers[4],
                                                            *_subscribers[5], *_subscribers[6]);
        _sync7->registerCallback(boost::bind(&PointCloudFusion::sync_pcl_callback, this, _1, _2, _3, _4, _5, _6, _7, null_pcl));
        break;
      }
      case(8):
      {
        _sync8 = std::make_shared<Synchronizer<sync_pcl_8> >(sync_pcl_8(10), *_subscribers[0], *_subscribers[1],
                                                            *_subscribers[2], *_subscribers[3], *_subscribers[4],
                                                            *_subscribers[5], *_subscribers[6], *_subscribers[7]);
        _sync8->registerCallback(boost::bind(&PointCloudFusion::sync_pcl_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
        break;
      }
        default:
          std::cout<<"default"<<std::endl;
          ROS_INFO("DEFAULT");
          break;
  }

  }// closing if
  _pcl_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud_fused", 5);

  //ROS_INFO("Done with constructor");
}// constuctor

  //helper function for transformation in callback
  void PointCloudFusion::transformation(sm_pcl2_type c, sm_pcl2& c_out){
      std::string frame_id = c->header.frame_id;
      try {
          ROS_INFO("std");
          _tf_listener.waitForTransform("base_link", frame_id, ros::Time(0), ros::Duration(0.2));

          _tf_listener.lookupTransform("base_link", frame_id, ros::Time(0), _stamped_transform);

      }

      catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          return;
      }
      catch (...){
          ROS_INFO("transformation failed");
          return;
      }
      if(_transforms.find(frame_id) == _transforms.end()){
          _transforms[frame_id] = std::make_shared<tf::Transform>(_stamped_transform);
      }
      pcl_ros::transformPointCloud( "base_link", *_transforms[frame_id], *c, c_out);
  }
  void PointCloudFusion::concatenate(sm_pcl2& c, sm_pcl2& combined)
  {
    if(combined.fields.size() == 0 or c.fields.size() == 0){
      return;
    }
    sm_pcl2 output;
    pcl::concatenatePointCloud(c, combined, output);
    combined = output;
    combined.header = c.header;
    combined.header.frame_id = "base_link";
  }
  // Then setup callback
  void PointCloudFusion::sync_pcl_callback(sm_pcl2_type c1, sm_pcl2_type c2,
          sm_pcl2_type c3, sm_pcl2_type c4, sm_pcl2_type c5, sm_pcl2_type c6,
          sm_pcl2_type c7, sm_pcl2_type c8)
  {
      ROS_INFO("Synchronizing PCLs...");
      sm_pcl2 combined_pcl;
      if(c1 == nullptr)
      {
          ROS_ERROR("Cloud 1 was null in callback.");
          return;
      }
      if (c2 == nullptr && c3 == nullptr && c4 == nullptr &&
            c5 == nullptr && c6 == nullptr && c7 == nullptr && c8 == nullptr)
      {
          return;
      }
      else
      {
          sm_pcl2 scratch_pcl;
          transformation(c1, combined_pcl);

          if(c2 != nullptr)
          {
            transformation(c2,scratch_pcl);
            concatenate(scratch_pcl, combined_pcl);
          }
          if(c3 != nullptr)
          {
            transformation(c3,scratch_pcl);
            concatenate(scratch_pcl, combined_pcl);
          }
          if(c4 != nullptr)
          {
            transformation(c4,scratch_pcl);
            concatenate(scratch_pcl, combined_pcl);
          }
          if(c5 != nullptr)
          {
            transformation(c5,scratch_pcl);
            concatenate(scratch_pcl, combined_pcl);
          }
          if(c6 != nullptr)
          {
            transformation(c6,scratch_pcl);
            concatenate(scratch_pcl, combined_pcl);
          }
          if(c7 != nullptr)
          {
            transformation(c7,scratch_pcl);
            concatenate(scratch_pcl, combined_pcl);
          }
          if(c8 != nullptr)
          {
            transformation(c8,scratch_pcl);
            concatenate(scratch_pcl, combined_pcl);
          }
      }
      _pcl_pub.publish(combined_pcl);

      ROS_INFO("Done Synchronizing PCLs.");

  }// call back



} // namespace pc_fusion
} // namespace velodyne_multi_fusion
