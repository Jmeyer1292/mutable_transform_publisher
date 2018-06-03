#include <ros/ros.h>
#include "mutable_transform_publisher/mutable_transform_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mutable_tf_publisher", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  mutable_transform_publisher::MutableTransformPublisher pub (nh);

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "foo";
  tf.child_frame_id = "bar";
  tf.transform.rotation.w = 1;
  pub.add(tf, ros::Duration(1.0));

  ros::spin();
}
