#ifndef MCT_MUTABLE_TRANSFORM_PUBLISHER_H
#define MCT_MUTABLE_TRANSFORM_PUBLISHER_H

#include "tf2_ros/transform_broadcaster.h"
#include "mutable_transform_publisher/publisher.h"

#include <memory>

namespace mutable_transform_publisher
{

class MutableTransformPublisher
{
public:
  MutableTransformPublisher();

  void add(const std::string& source, const std::string& target,
           ros::Duration period, const geometry_msgs::Transform& initial_tf);

private:
  std::string makeKey(const std::string& source, const std::string& target) const;

  tf2_ros::TransformBroadcaster broadcaster_;
  std::map<std::string, Publisher> pub_map_;
};

}

#endif // MCT_MUTABLE_TRANSFORM_PUBLISHER_H
