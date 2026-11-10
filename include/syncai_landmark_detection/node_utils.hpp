#ifndef SYNCAI_LANDMARK_DETECTION__NODE_UTILS_HPP_
#define SYNCAI_LANDMARK_DETECTION__NODE_UTILS_HPP_

// The two parameter helpers the plugin servers in SyncAI-Robot-Workspace use,
// with the same names and behaviour as syncai_util's (node_utils.hpp there),
// so a plugin here reads like a planner or controller plugin there. Copied
// rather than depended on: syncai_util is a workspace package that needs the
// whole nav stack (syncai_costmap, ...) to build, and this repo builds on its
// own against syncai_common only.

#include <cstdio>
#include <cstdlib>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace syncai_landmark_detection
{

/**
 * @brief Declares a parameter with a default value if it was not declared yet.
 * @param node A node in which the parameter is declared
 * @param param_name The name of the parameter
 * @param default_value The value to initialise it with
 * @param parameter_descriptor Parameter descriptor (optional)
 */
template <typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node, const std::string & param_name, const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

/**
 * @brief Declares a parameter of a given type, with no default, if it was not
 * declared yet. Reading it before it is set throws
 * rclcpp::exceptions::ParameterUninitializedException.
 * @param node A node in which the parameter is declared
 * @param param_name The name of the parameter
 * @param param_type The type of the parameter
 * @param parameter_descriptor Parameter descriptor (optional)
 */
template <typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node, const std::string & param_name, const rclcpp::ParameterType & param_type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}

/**
 * @brief The pluginlib type of a plugin instance: the value of its
 * `<plugin_name>.plugin` parameter. Exits the process when it is not set, as
 * syncai_util's does: a server without its plugin cannot do anything.
 * @param node The node that hosts the plugin
 * @param plugin_name The plugin instance's name (its parameter namespace)
 */
template <typename NodeT>
std::string get_plugin_type_param(NodeT node, const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::PARAMETER_STRING);
  std::string plugin_type;
  try {
    if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
      RCLCPP_FATAL(
        node->get_logger(), "Can not get 'plugin' param value for %s", plugin_name.c_str());
      exit(-1);
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_FATAL(node->get_logger(), "'plugin' param not defined for %s", plugin_name.c_str());
    exit(-1);
  }

  return plugin_type;
}

/**
 * @brief A duration for a log line or an error_msg, "0.35 s". Not from
 * syncai_util: the server and the plugins used to carry their own copies, at
 * different precisions. Two decimals, because ages are compared against a
 * max_age of tenths (0.3 s).
 */
inline std::string formatSeconds(double s)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f s", s);
  return buffer;
}

}  // namespace syncai_landmark_detection

#endif  // SYNCAI_LANDMARK_DETECTION__NODE_UTILS_HPP_
