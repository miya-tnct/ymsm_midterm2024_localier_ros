#ifndef YMSM_MIDTERM2024_LOCALIZER_LOCALIZER_NODE_H_
#define YMSM_MIDTERM2024_LOCALIZER_LOCALIZER_NODE_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace ymsm_midterm2024_localizer::localizer
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  void initialize_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr initialpose_msg);


  geometry_msgs::TransformStamped tf_msg_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listerner_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  ros::Subscriber initialpose_subscriber_;
};

}

#endif