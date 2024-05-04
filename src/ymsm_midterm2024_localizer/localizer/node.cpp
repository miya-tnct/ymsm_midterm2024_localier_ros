#include "ymsm_midterm2024_localizer/localizer/node.h"

#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ymsm_midterm2024_localizer::localizer
{

Node::Node() :
  ros::NodeHandle(),
  tf_msg_([](){
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "odom";
    tf_msg.transform.rotation.w = 1;
    return tf_msg;
  }()),
  tf_buffer_(),
  tf_listerner_(tf_buffer_),
  static_tf_broadcaster_(),
  initialpose_subscriber_(this->subscribe("initialpose", 1, &Node::initialize_pose, this))
{
  static_tf_broadcaster_.sendTransform(tf_msg_);
}


void Node::initialize_pose(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr initialpose_msg)
{
  geometry_msgs::TransformStamped base_link_tf_msg, initialpose_parent_tf_msg;
  try {
    initialpose_parent_tf_msg = tf_buffer_.lookupTransform(
      "map", initialpose_msg->header.frame_id, ros::Time(0));
  }
  catch (...) {
    ROS_WARN("Cannot transform %s to %s", "map", initialpose_msg->header.frame_id.c_str());
    return;
  }
  try {
    base_link_tf_msg = tf_buffer_.lookupTransform(
      "odom", "base_link", ros::Time(0));
  }
  catch (...) {
    ROS_WARN("Cannot transform %s to %s", "odom", "base_link");
    return;
  }
  tf2::Transform initialpose_tf, base_link_tf, initialpose_parent_tf;
  tf2::fromMsg(initialpose_msg->pose.pose, initialpose_tf);
  tf2::fromMsg(base_link_tf_msg.transform, base_link_tf);
  tf2::fromMsg(initialpose_parent_tf_msg.transform, initialpose_parent_tf);
  auto odom_tf = initialpose_parent_tf * initialpose_tf * base_link_tf.inverse();

  tf_msg_.header.stamp = initialpose_msg->header.stamp;
  tf_msg_.header.seq = initialpose_msg->header.seq;
  tf_msg_.transform = tf2::toMsg(odom_tf);
  static_tf_broadcaster_.sendTransform(tf_msg_);
}

}