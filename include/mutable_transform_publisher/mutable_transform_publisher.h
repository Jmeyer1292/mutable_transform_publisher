#ifndef MCT_MUTABLE_TRANSFORM_PUBLISHER_H
#define MCT_MUTABLE_TRANSFORM_PUBLISHER_H

#include "tf2_ros/transform_broadcaster.h"
#include "mutable_transform_publisher/publisher.h"
#include "mutable_transform_publisher/SetTransform.h"

#include <memory>

namespace mutable_transform_publisher
{

class MutableTransformPublisher
{
public:
  MutableTransformPublisher(ros::NodeHandle& nh);

  bool add(const geometry_msgs::TransformStamped& transform, ros::Duration period);

  std::vector<geometry_msgs::TransformStamped> getAllTransforms() const;

private:
  bool setTransformCallback(SetTransformRequest& req, SetTransformResponse& res);

  Publisher* findPublisher(const std::string& source, const std::string& target) const;

  std::string makeKey(const std::string& source, const std::string& target) const;

  bool validate(const geometry_msgs::TransformStamped& t) const;

  tf2_ros::TransformBroadcaster broadcaster_;
  ros::ServiceServer set_transform_server_;
  std::map<std::string, std::unique_ptr<Publisher>> pub_map_;
};

}

#endif // MCT_MUTABLE_TRANSFORM_PUBLISHER_H
