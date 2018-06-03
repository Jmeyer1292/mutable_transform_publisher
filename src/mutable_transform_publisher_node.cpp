#include <ros/ros.h>
#include "mutable_transform_publisher/mutable_transform_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mutable_tf_publisher", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  mutable_transform_publisher::MutableTransformPublisher pub (nh);

  geometry_msgs::Transform tf;
  tf.rotation.w = 1.0;
  pub.add("foo","bar", ros::Duration(1.0), tf);

  ros::spin();
}
