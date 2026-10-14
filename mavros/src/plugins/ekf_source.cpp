/*
 * Copyright 2026 GarudaNav VIO Safety contributors.
 *
 * This file is part of the mavros package and is subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Read-only decoder for the pinned Lua VSRC source witness.
 */
#include <cmath>
#include <cstdint>
#include <limits>

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros_msgs/msg/ekf_source_status.hpp"

namespace mavros
{
namespace std_plugins
{

class EkfSourcePlugin : public plugin::Plugin
{
public:
  explicit EkfSourcePlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "ekf_source")
  {
    source_pub_ = node->create_publisher<mavros_msgs::msg::EkfSourceStatus>(
      "~/ekf_source", rclcpp::QoS(10).reliable());
  }

  Subscriptions get_subscriptions() override
  {
    return {make_handler(&EkfSourcePlugin::handle_named_value_float)};
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::EkfSourceStatus>::SharedPtr source_pub_;
  std::uint64_t receive_sequence_{0};
  std::uint64_t connection_epoch_{1};
  std::uint32_t last_time_boot_ms_{0};
  bool have_time_boot_{false};

  void handle_named_value_float(
    const mavlink::mavlink_message_t * msg,
    mavlink::common::msg::NAMED_VALUE_FLOAT & value,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (mavlink::to_string(value.name) != "VSRC") {
      return;
    }
    if (have_time_boot_ && value.time_boot_ms < last_time_boot_ms_) {
      ++connection_epoch_;
    }
    last_time_boot_ms_ = value.time_boot_ms;
    have_time_boot_ = true;

    mavros_msgs::msg::EkfSourceStatus output;
    output.header.stamp = uas->synchronise_stamp(value.time_boot_ms);
    output.time_boot_ms = value.time_boot_ms;
    output.source_system = msg == nullptr ? 0U : msg->sysid;
    output.source_component = msg == nullptr ? 0U : msg->compid;
    output.receive_sequence = ++receive_sequence_;
    output.connection_epoch = connection_epoch_;
    constexpr double base_v1 = 1048576.0;
    constexpr double base_v2 = 2097152.0;
    constexpr double base_v3 = 4194304.0;
    const double rounded = std::round(static_cast<double>(value.value));
    const bool integral = std::abs(static_cast<double>(value.value) - rounded) <= 0.5;
    output.known = false;
    output.source1_ekf_independent = false;
    output.gps_measurement_fresh = false;
    if (std::isfinite(value.value) && integral && rounded >= base_v3) {
      const double remainder = rounded - base_v3;
      const bool in_range = remainder >= 0.0 &&
                            remainder <= 65535.0 * 64.0 + 2.0 * 16.0 + 2.0 * 4.0 + 3.0;
      output.schema_version = 3;
      if (in_range) {
        const auto packed = static_cast<std::uint64_t>(remainder);
        output.witness_sequence = static_cast<std::uint16_t>((packed / 64U) & 0xffffU);
        output.gps_index = static_cast<std::uint8_t>((packed / 16U) & 0x03U);
        const auto source_index = static_cast<std::uint8_t>((packed / 4U) & 0x03U);
        output.source_set = static_cast<std::uint8_t>(source_index + 1U);
        output.source1_ekf_independent = ((packed / 2U) & 1U) != 0U;
        output.gps_measurement_fresh = (packed & 1U) != 0U;
        output.known = source_index <= 2U && output.gps_index <= 2U;
      }
    } else if (std::isfinite(value.value) && integral && rounded >= base_v2) {
      const double remainder = rounded - base_v2;
      const bool in_range = remainder >= 0.0 &&
                            remainder <= 65535.0 * 32.0 + 2.0 * 8.0 + 2.0 * 2.0 + 1.0;
      output.schema_version = 2;
      if (in_range) {
        const auto packed = static_cast<std::uint64_t>(remainder);
        output.witness_sequence = static_cast<std::uint16_t>((packed / 32U) & 0xffffU);
        output.gps_index = static_cast<std::uint8_t>((packed / 8U) & 0x03U);
        const auto source_index = static_cast<std::uint8_t>((packed / 2U) & 0x03U);
        output.source_set = static_cast<std::uint8_t>(source_index + 1U);
        output.source1_ekf_independent = (packed & 0x01U) != 0U;
        output.known = source_index <= 2U && output.gps_index <= 2U;
      }
    } else if (std::isfinite(value.value) && integral && rounded >= base_v1) {
      const double remainder = rounded - base_v1;
      const bool in_range = remainder >= 0.0 && remainder <= 65535.0 * 16.0 + 3.0;
      output.schema_version = 1;
      if (in_range) {
        const auto packed = static_cast<std::uint64_t>(remainder);
        output.witness_sequence = static_cast<std::uint16_t>((packed / 16U) & 0xffffU);
        output.gps_index = static_cast<std::uint8_t>((packed / 4U) & 0x03U);
        const auto source_index = static_cast<std::uint8_t>(packed & 0x03U);
        output.source_set = static_cast<std::uint8_t>(source_index + 1U);
        output.known = source_index <= 2U && output.gps_index <= 2U;
      }
    }
    if (!output.known) {
      output.reason_mask = 1U;
    }
    source_pub_->publish(output);
  }
};

}  // namespace std_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::EkfSourcePlugin)
