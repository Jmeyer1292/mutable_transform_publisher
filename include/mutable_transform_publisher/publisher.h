#ifndef MCT_PUBLISHER_H
#define MCT_PUBLISHER_H

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/timer.h>

namespace mutable_transform_publisher
{

class Publisher
{
public:
  Publisher(const std::string& source, const std::string& target, ros::Duration period,
            const geometry_msgs::Transform& init_tf, tf2_ros::TransformBroadcaster& broadcaster);

  geometry_msgs::TransformStamped setTransform(const geometry_msgs::Transform& t);

  geometry_msgs::TransformStamped getTransform() const;

private:
  void onPublishTimeout(const ros::TimerEvent& e);

  std::string source_;
  std::string target_;
  geometry_msgs::TransformStamped tf_;
  ros::Timer pub_timer_;
  tf2_ros::TransformBroadcaster& broadcaster_;
};

}

#endif // MCT_PUBLISHER_H
