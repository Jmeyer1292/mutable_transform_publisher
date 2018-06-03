#include "mutable_transform_publisher/mutable_transform_publisher.h"

mutable_transform_publisher::MutableTransformPublisher::MutableTransformPublisher()
{
}

void mutable_transform_publisher::MutableTransformPublisher::add(const std::string& source, const std::string& target,
                                                                 ros::Duration period,
                                                                 const geometry_msgs::Transform& initial_tf)
{
  const auto key = makeKey(source, target);
  Publisher pub (source, target, period, initial_tf, broadcaster_);
  pub_map_.emplace(key, std::move(pub));
}

std::string mutable_transform_publisher::MutableTransformPublisher::makeKey(const std::string& source,
                                                                            const std::string& target) const
{
  return source + target;
}
