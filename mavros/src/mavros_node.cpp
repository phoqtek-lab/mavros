/*
 * Copyright 2013,2014,2015,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief MAVROS Node
 * @file mavros_node.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */

#include <pthread.h>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "mavros/mavros_router.hpp"
#include "mavros/mavros_uas.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * MAVROS Node is a transition helper, a component loader preconfigured
 * to work similar to mavros v1.
 */
int main(int argc, char * argv[])
{
  sigset_t termination_signals;
  sigemptyset(&termination_signals);
  sigaddset(&termination_signals, SIGINT);
  sigaddset(&termination_signals, SIGTERM);
  const int mask_result = pthread_sigmask(SIG_BLOCK, &termination_signals, nullptr);
  if (mask_result != 0) {
    std::fprintf(stderr, "failed to block ROS termination signals: %d\n", mask_result);
    return 1;
  }

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto context = rclcpp::contexts::get_global_default_context();
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  auto exec = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
    executor_options, 2);

  rclcpp::NodeOptions options;
  options.context(context);
  // options.use_intra_process_comms(true);

  std::string fcu_url, gcs_url, uas_url;
  std::string base_link_frame_id, odom_frame_id, map_frame_id;
  int tgt_system = 1, tgt_component = 1;

  auto node = std::make_shared<rclcpp::Node>("mavros_node", options);
  exec->add_node(node);

  node->declare_parameter("fcu_url", fcu_url);
  node->declare_parameter("gcs_url", gcs_url);
  node->declare_parameter("tgt_system", tgt_system);
  node->declare_parameter("tgt_component", tgt_component);
  node->declare_parameter("base_link_frame", base_link_frame_id);
  node->declare_parameter("map_frame", map_frame_id);
  node->declare_parameter("odom_frame", odom_frame_id);

  node->get_parameter("fcu_url", fcu_url);
  node->get_parameter("gcs_url", gcs_url);
  node->get_parameter("tgt_system", tgt_system);
  node->get_parameter("tgt_component", tgt_component);
  node->get_parameter("base_link_frame", base_link_frame_id);
  node->get_parameter("map_frame", map_frame_id);
  node->get_parameter("odom_frame", odom_frame_id);

  uas_url = mavros::utils::format("/uas%d", tgt_system);

  RCLCPP_INFO(node->get_logger(), "Starting mavros_node container");
  RCLCPP_INFO(node->get_logger(), "FCU URL: %s", fcu_url.c_str());
  RCLCPP_INFO(node->get_logger(), "GCS URL: %s", gcs_url.c_str());
  RCLCPP_INFO(node->get_logger(), "UAS Prefix: %s", uas_url.c_str());

  RCLCPP_INFO(node->get_logger(), "Starting mavros router node");
  auto router_node = std::make_shared<mavros::router::Router>(options, "mavros_router");
  exec->add_node(router_node);

  {
    std::vector<rclcpp::Parameter> router_params{};

    if (fcu_url != "") {
      router_params.emplace_back("fcu_urls", std::vector<std::string>{fcu_url});
    }
    if (gcs_url != "") {
      router_params.emplace_back("gcs_urls", std::vector<std::string>{gcs_url});
    }
    router_params.emplace_back("uas_urls", std::vector<std::string>{uas_url});

    router_node->set_parameters(router_params);
  }

  RCLCPP_INFO(node->get_logger(), "Starting mavros uas node");
  auto uas_node = std::make_shared<mavros::uas::UAS>(
    options, "mavros", uas_url, tgt_system,
    tgt_component);
  exec->add_node(uas_node);

  {
    std::vector<rclcpp::Parameter> uas_params{};

    if (base_link_frame_id != "") {
      uas_params.emplace_back("base_link_frame_id", base_link_frame_id);
    }

    if (odom_frame_id != "") {
      uas_params.emplace_back("odom_frame_id", odom_frame_id);
    }

    if (map_frame_id != "") {
      uas_params.emplace_back("map_frame_id", map_frame_id);
    }
    uas_node->set_parameters(uas_params);
  }

  std::thread termination_thread([&exec, &termination_signals]() {
    int received_signal = 0;
    int wait_result = 0;
    do {
      wait_result = sigwait(&termination_signals, &received_signal);
    } while (wait_result == EINTR);
    exec->cancel();
  });

  exec->spin();
  termination_thread.join();
  exec->cancel();
  exec->remove_node(uas_node);
  exec->remove_node(router_node);
  exec->remove_node(node);
  uas_node->shutdown_plugins();
  uas_node.reset();
  router_node->shutdown_endpoints();
  router_node.reset();
  node.reset();
  exec.reset();

  if (context->is_valid()) {
    context->shutdown("MAVROS executor stopped");
  }
  context.reset();
  return 0;
}
