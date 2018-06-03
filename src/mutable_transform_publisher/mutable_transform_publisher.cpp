#include "mutable_transform_publisher/mutable_transform_publisher.h"

static bool isNormalized(const geometry_msgs::Quaternion& q, const double eps = 1e-6)
{
  const auto sum = q.w + q.x + q.y + q.z;
  return std::abs(1.0 - sum) < eps;
}

mutable_transform_publisher::MutableTransformPublisher::MutableTransformPublisher(ros::NodeHandle& nh)
{
  set_transform_server_ = nh.advertiseService("set_transform", &MutableTransformPublisher::setTransformCallback, this);
}

void mutable_transform_publisher::MutableTransformPublisher::add(const geometry_msgs::TransformStamped& transform,
                                                                 ros::Duration period)
{
  if (!validate(transform))
  {
    ROS_WARN_STREAM("Transform push rejected: " << transform);
    return;
  }

  const auto& source = transform.header.frame_id;
  const auto target = transform.child_frame_id;
  const auto key = makeKey(source, target);
  std::unique_ptr<Publisher> pub (new Publisher(source, target, period, transform.transform, broadcaster_));
//  auto it = pub_map_.emplace(key, std::move(pub));
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
    add(req.transform, ros::Duration(1.0));
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

bool mutable_transform_publisher::MutableTransformPublisher::validate(const geometry_msgs::TransformStamped& t) const
{
  if (t.child_frame_id.empty())
  {
    ROS_WARN_STREAM("transform's child_frame_id is empty");
    return false;
  }

  if (t.header.frame_id.empty())
  {
    ROS_WARN_STREAM("transform's header.frame_id is empty");
    return false;
  }

  if (!isNormalized(t.transform.rotation))
  {
    ROS_WARN_STREAM("transform quaternion is not normalized");
    return false;
  }

  return true;
}
