#include "mutable_transform_publisher/mutable_transform_publisher.h"

mutable_transform_publisher::MutableTransformPublisher::MutableTransformPublisher(ros::NodeHandle& nh)
{
  set_transform_server_ = nh.advertiseService("set_transform", &MutableTransformPublisher::setTransformCallback, this);
}

void mutable_transform_publisher::MutableTransformPublisher::add(const std::string& source, const std::string& target,
                                                                 ros::Duration period,
                                                                 const geometry_msgs::Transform& initial_tf)
{
  const auto key = makeKey(source, target);
  std::unique_ptr<Publisher> pub (new Publisher(source, target, period, initial_tf, broadcaster_));
  pub_map_.emplace(key, std::move(pub));
}

bool mutable_transform_publisher::MutableTransformPublisher::setTransformCallback(SetTransformRequest& req,
                                                                                  SetTransformResponse& res)
{
  auto* pub = findPublisher(req.transform.header.frame_id, req.transform.child_frame_id);
  if (pub)
  {
    res.was_replaced = true;
    res.old_transform.header.frame_id = req.transform.header.frame_id;
    res.old_transform.header.stamp = ros::Time::now();
    res.old_transform.child_frame_id = req.transform.child_frame_id;
    res.old_transform.transform = pub->getTransform();
    pub->setTransform(req.transform.transform);
  }
  else
  {
    res.was_replaced = false;
    add(req.transform.header.frame_id, req.transform.child_frame_id, ros::Duration(1.0), req.transform.transform);
  }

  return true;
}

mutable_transform_publisher::Publisher*
mutable_transform_publisher::MutableTransformPublisher::findPublisher(const std::string& source,
                                                                      const std::string& target) const
{
  const auto key = makeKey(source, target);
  auto it = pub_map_.find(key);
  if (it == pub_map_.end())
  {
    return nullptr;
  }
  else
  {
    return it->second.get();
  }
}

std::string mutable_transform_publisher::MutableTransformPublisher::makeKey(const std::string& source,
                                                                            const std::string& target) const
{
  return source + target;
}
