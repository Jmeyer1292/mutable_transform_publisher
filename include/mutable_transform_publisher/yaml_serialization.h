#ifndef MTP_YAML_SERIALIZATION_H
#define MTP_YAML_SERIALIZATION_H

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/TransformStamped.h>

namespace mutable_transform_publisher
{

bool deserialize(const std::string& path, std::vector<geometry_msgs::TransformStamped>& tfs);

bool serialize(const std::string& path, const std::vector<geometry_msgs::TransformStamped>& tfs);

}

#endif // MTP_YAML_SERIALIZATION_H
