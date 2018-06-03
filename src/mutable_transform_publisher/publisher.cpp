#include "mutable_transform_publisher/publisher.h"
#include <ros/node_handle.h>

mutable_transform_publisher::Publisher::Publisher(const std::string& source, const std::string& target,
                                                  ros::Duration period, const geometry_msgs::Transform& init_tf,
                                                  tf2_ros::TransformBroadcaster& broadcaster)
  : source_(source)
  , target_(target)
  , broadcaster_(broadcaster)
{
  tf_.transform = init_tf;
  tf_.header.frame_id = source;
  tf_.child_frame_id = target;

  ros::NodeHandle nh;
  pub_timer_ = nh.createTimer(period, &Publisher::onPublishTimeout, this);
}

geometry_msgs::TransformStamped mutable_transform_publisher::Publisher::setTransform(const geometry_msgs::Transform& t)
{
  const auto copy = tf_;
  tf_.transform = t;
  return copy;
}

geometry_msgs::Transform mutable_transform_publisher::Publisher::getTransform() const
{
  return tf_.transform;
}

void mutable_transform_publisher::Publisher::onPublishTimeout(const ros::TimerEvent& e)
{
  tf_.header.stamp = e.current_real;
  broadcaster_.sendTransform(tf_);
}
