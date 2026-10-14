/*
 * Copyright 2026 GarudaNav VIO Safety contributors.
 *
 * This file is part of the mavros package and is subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Read-only full EKF_STATUS_REPORT projection for the safety supervisor.
 */
#include <cmath>
#include <cstdint>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/ekf_safety_status.hpp"

namespace mavros
{
namespace std_plugins
{
class EkfSafetyPlugin : public plugin::Plugin
{
public:
  explicit EkfSafetyPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "ekf_safety"), connection_epoch_(1)
  {
    ekf_pub_ = node->create_publisher<mavros_msgs::msg::EkfSafetyStatus>(
      "~/ekf_safety", rclcpp::QoS(10).reliable());
  }

  Subscriptions get_subscriptions() override
  {
    return {make_handler(&EkfSafetyPlugin::handle_ekf_status_report)};
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::EkfSafetyStatus>::SharedPtr ekf_pub_;
  std::uint64_t receive_sequence_{0};
  std::uint64_t connection_epoch_;

  void handle_ekf_status_report(
    const mavlink::mavlink_message_t * msg,
    mavlink::ardupilotmega::msg::EKF_STATUS_REPORT & status,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    mavros_msgs::msg::EkfSafetyStatus output;
    output.header.stamp = node->now();
    output.flags = status.flags;
    output.velocity_variance = status.velocity_variance;
    output.pos_horiz_variance = status.pos_horiz_variance;
    output.pos_vert_variance = status.pos_vert_variance;
    output.compass_variance = status.compass_variance;
    output.terrain_alt_variance = status.terrain_alt_variance;
    output.airspeed_variance = status.airspeed_variance;
    output.variances_valid = std::isfinite(status.velocity_variance) &&
      std::isfinite(status.pos_horiz_variance) && std::isfinite(status.pos_vert_variance) &&
      std::isfinite(status.compass_variance) && std::isfinite(status.terrain_alt_variance) &&
      std::isfinite(status.airspeed_variance);
    output.time_boot_ms = 0;
    output.source_system = msg == nullptr ? 0U : msg->sysid;
    output.source_component = msg == nullptr ? 0U : msg->compid;
    output.receive_sequence = ++receive_sequence_;
    output.connection_epoch = connection_epoch_;
    ekf_pub_->publish(output);
  }
};

}  // namespace std_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::EkfSafetyPlugin)
