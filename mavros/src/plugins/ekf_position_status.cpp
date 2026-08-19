/*
 * Copyright 2026 Pioneer Bridge integration.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief EKF position status plugin
 * @file ekf_position_status.cpp
 *
 * @addtogroup plugin
 * @{
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/ekf_position_status.hpp"

namespace mavros
{
namespace std_plugins
{
using utils::enum_value;

/**
 * @brief EKF position status plugin
 * @plugin ekf_position_status
 *
 * Publishes a bounded horizontal-position subset of the ArduPilot-specific
 * EKF_STATUS_REPORT message (ardupilotmega dialect). Does not surface the
 * full flag/variance set, and does not accept any command input -- this
 * plugin is read-only.
 */
class EkfPositionStatusPlugin : public plugin::Plugin
{
public:
  explicit EkfPositionStatusPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "ekf_position_status")
  {
    auto state_qos = rclcpp::QoS(10).transient_local();

    ekf_position_status_pub = node->create_publisher<mavros_msgs::msg::EkfPositionStatus>(
      "~/ekf_position_status", state_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&EkfPositionStatusPlugin::handle_ekf_status_report),
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::EkfPositionStatus>::SharedPtr ekf_position_status_pub;

  void handle_ekf_status_report(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::EKF_STATUS_REPORT & status,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    using EKF = mavlink::ardupilotmega::EKF_STATUS_FLAGS;

    auto ekf_position_status = mavros_msgs::msg::EkfPositionStatus();
    ekf_position_status.header.stamp = node->now();
    ekf_position_status.pos_horiz_abs =
      (status.flags & enum_value(EKF::POS_HORIZ_ABS)) != 0;
    ekf_position_status.pos_horiz_rel =
      (status.flags & enum_value(EKF::POS_HORIZ_REL)) != 0;
    ekf_position_status.uninitialized =
      (status.flags & enum_value(EKF::UNINITIALIZED)) != 0;

    ekf_position_status_pub->publish(ekf_position_status);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::EkfPositionStatusPlugin)
