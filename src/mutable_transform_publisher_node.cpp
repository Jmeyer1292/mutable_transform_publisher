#include <ros/ros.h>
#include "mutable_transform_publisher/mutable_transform_publisher.h"
#include "mutable_transform_publisher/yaml_serialization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mutable_tf_publisher", ros::init_options::AnonymousName);
  ros::NodeHandle nh, pnh ("~");

//  std::string yaml_path;
//  if (!pnh.getParam("yaml_path", yaml_path))
//  {
//    ROS_FATAL_STREAM("mutable_transform_publisher requires the 'yaml_path' private parameter to be set");
//    return 1;
//  }

  // Create the publisher
  mutable_transform_publisher::MutableTransformPublisher pub (nh);

  // Parse the yaml file
//  std::vector<geometry_msgs::TransformStamped> tfs;
//  if (!mutable_transform_publisher::deserialize(yaml_path, tfs))
//  {

//  }

  ros::spin();

//  const auto new_tfs = pub.getAllTransforms();

//  if (!mutable_transform_publisher::serialize(yaml_path, new_tfs))
//  {
//    std::cerr << "mutable_transform_publisher: Unable to serialize transforms to " << yaml_path << "\n";
//    return 1;
//  }

  return 0;
}
