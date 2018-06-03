#include "mutable_transform_publisher/yaml_serialization.h"
#include <yaml-cpp/yaml.h>
#include <ros/console.h>
#include <fstream>

static geometry_msgs::TransformStamped parseTransform(const YAML::Node& n)
{
  geometry_msgs::TransformStamped t;
  t.header.frame_id = n["parent"].as<std::string>();
  t.child_frame_id = n["child"].as<std::string>();

  t.transform.translation.x = n["x"].as<double>();
  t.transform.translation.y = n["y"].as<double>();
  t.transform.translation.z = n["z"].as<double>();

  t.transform.rotation.x = n["qx"].as<double>();
  t.transform.rotation.y = n["qy"].as<double>();
  t.transform.rotation.z = n["qz"].as<double>();
  t.transform.rotation.w = n["qw"].as<double>();

  return t;
}

static YAML::Node convertTransform(const geometry_msgs::TransformStamped& t)
{
  YAML::Node n;
  n["parent"] = t.header.frame_id;
  n["child"] = t.child_frame_id;
  n["x"] = t.transform.translation.x;
  n["y"] = t.transform.translation.y;
  n["z"] = t.transform.translation.z;
  n["qx"] = t.transform.rotation.x;
  n["qy"] = t.transform.rotation.y;
  n["qz"] = t.transform.rotation.z;
  n["qw"] = t.transform.rotation.w;
  return n;
}

bool mutable_transform_publisher::deserialize(const std::string& path, std::vector<geometry_msgs::TransformStamped>& tfs)
{
  try
  {
    YAML::Node root = YAML::LoadFile(path);
    for (std::size_t i = 0; i < root.size(); ++i)
    {
      const auto t = parseTransform(root[i]);
      tfs.push_back(t);
    }

    return true;
  }
  catch (const YAML::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
}

bool mutable_transform_publisher::serialize(const std::string& path, const std::vector<geometry_msgs::TransformStamped>& tfs)
{
  YAML::Node root;
  for (const auto& t : tfs)
  {
    root.push_back(convertTransform(t));
  }

  std::ofstream ofh (path);
  ofh << root;
  return true;
}
