#pragma once

#include <algorithm>
#include <cctype>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace AutoFlight
{
inline rclcpp::QoS buildTargetQos(int depth, std::string reliability, std::string durability)
{
  rclcpp::QoS qos{rclcpp::KeepLast(std::max(1, depth))};

  std::transform(
    reliability.begin(), reliability.end(), reliability.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (reliability == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }

  std::transform(
    durability.begin(), durability.end(), durability.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (durability == "transient_local") {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }

  return qos;
}
} // namespace AutoFlight
