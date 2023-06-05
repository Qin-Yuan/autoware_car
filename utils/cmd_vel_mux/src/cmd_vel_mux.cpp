/**
 * @file /src/cmd_vel_mux_nodelet.cpp
 *
 * @brief  Implementation for the command velocity multiplexer
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "cmd_vel_mux/cmd_vel_mux.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{

namespace
{
std::vector<std::string> stringSplit(const std::string & str,
                                     const std::string & splitter)
{
  std::vector<std::string> ret;
  size_t next = 0;
  size_t current = next;

  if (splitter.empty())
  {
    // If the splitter is blank, just return the original
    ret.push_back(str);
    return ret;
  }

  while (next != std::string::npos)
  {
    next = str.find(splitter, current);
    ret.push_back(str.substr(current, next - current));
    current = next + splitter.length();
  }

  return ret;
}

}  // namespace

/*****************************************************************************
 ** Implementation
 *****************************************************************************/
const std::string CmdVelMux::VACANT = "empty";

CmdVelMux::CmdVelMux(rclcpp::NodeOptions options) : rclcpp::Node("cmd_vel_mux", options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)), allowed_(VACANT)
{
  std::map<std::string, rclcpp::Parameter> parameters;
  // Check if there are loaded parameters from config file besides sim_time_used
  if (!get_parameters("subscribers", parameters) || parameters.size() < 1)
  {
    RCLCPP_WARN(get_logger(), "No subscribers configured!");
  }
  else
  {
    used_priorities_.clear();
    std::map<std::string, ParameterValues> parsed_parameters = parseFromParametersMap(parameters);
    if (parsed_parameters.size() == 0)
    {
      // We ran into some kind of error while configuring, quit
      throw std::runtime_error("Invalid parameters");
    }
    if (!parametersAreValid(parsed_parameters))
    {
      throw std::runtime_error("Incomplete parameters");
    }
    configureFromParameters(parsed_parameters);
  }

  param_cb_ =
    add_on_set_parameters_callback(std::bind(&CmdVelMux::parameterUpdate, this,
      std::placeholders::_1));

  output_topic_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  RCLCPP_DEBUG(get_logger(), "CmdVelMux : subscribe to output topic 'cmd_vel'");

  active_subscriber_pub_ = this->create_publisher<std_msgs::msg::String>("active", rclcpp::QoS(1).transient_local()); // latched topic

  // Notify the world that right now nobody is publishing on cmd_vel yet
  auto active_msg = std::make_unique<std_msgs::msg::String>();
  active_msg->data = "idle";
  active_subscriber_pub_->publish(std::move(active_msg));

  RCLCPP_DEBUG(get_logger(), "CmdVelMux : successfully initialized");
}

bool CmdVelMux::parametersAreValid(const std::map<std::string, ParameterValues> & parameters)
{
  for (const std::pair<std::string, ParameterValues> & parameter : parameters)
  {
    if (parameter.second.topic.empty())
    {
      RCLCPP_WARN(get_logger(), "Empty topic for '%s'", parameter.first.c_str());
      return false;
    }
    if (parameter.second.timeout < 0.0)
    {
      RCLCPP_WARN(get_logger(), "Missing timeout for '%s', ignoring", parameter.first.c_str());
      return false;
    }
    if (parameter.second.priority < 0)
    {
      RCLCPP_WARN(get_logger(), "Missing priority for '%s', ignoring", parameter.first.c_str());
      return false;
    }
    if (parameter.second.short_desc.empty())
    {
      RCLCPP_WARN(get_logger(), "Empty short_desc for '%s', ignoring", parameter.first.c_str());
      return false;
    }
  }

  return true;
}

void CmdVelMux::configureFromParameters(const std::map<std::string, ParameterValues> & parameters)
{
  std::map<std::string, std::shared_ptr<CmdVelSub>> new_map;

  for (const std::pair<std::string, ParameterValues> & parameter : parameters)
  {
    const std::string & key = parameter.first;
    const ParameterValues & parameter_values = parameter.second;
    // Check if parameter subscriber has all its necessary values
    if (map_.count(key) != 0)
    {
      // For names already in the subscribers map, retain current object so we don't re-subscribe to the topic
      new_map[key] = map_[key];
    }
    else
    {
      new_map[key] = std::make_shared<CmdVelSub>();
    }

    // update existing or new object with the new configuration

    new_map[key]->name_ = key;
    new_map[key]->priority_ = parameter_values.priority;
    new_map[key]->short_desc_ = parameter_values.short_desc;

    if (parameter_values.topic != new_map[key]->topic_)
    {
      // Shutdown the topic if the name has changed so it gets recreated on configuration reload
      // In the case of new subscribers, topic is empty and shutdown has just no effect
      new_map[key]->topic_ = parameter_values.topic;
      new_map[key]->sub_ = nullptr;
    }

    if (parameter_values.timeout != new_map[key]->timeout_)
    {
      // Change timer period if the timeout changed
      new_map[key]->timeout_ = parameter_values.timeout;
      new_map[key]->timer_ = nullptr;
    }
  }

  // Take down the deleted subscriber if it was the one being used as source
  if (allowed_ != VACANT && new_map.count(allowed_) == 0)
  {
    allowed_ = VACANT;
    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    auto active_msg = std::make_unique<std_msgs::msg::String>();
    active_msg->data = "idle";
    active_subscriber_pub_->publish(std::move(active_msg));
  }

  map_ = new_map;

  // (Re)create subscribers whose topic is invalid: new ones and those with changed names
  double longest_timeout = 0.0;
  for (std::pair<const std::string, std::shared_ptr<CmdVelSub>> & m : map_)
  {
    const std::string & key = m.first;
    const std::shared_ptr<CmdVelSub> & values = m.second;
    if (!values->sub_)
    {
      values->sub_ = this->create_subscription<geometry_msgs::msg::Twist>(values->topic_, 10, [this, key](const geometry_msgs::msg::Twist::SharedPtr msg){cmdVelCallback(msg, key);});
      RCLCPP_DEBUG(get_logger(), "CmdVelMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
                   values->name_.c_str(), values->topic_.c_str(),
                   values->priority_, values->timeout_);
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "CmdVelMux : no need to re-subscribe to input topic '%s'", values->topic_.c_str());
    }

    if (!values->timer_)
    {
      // Create (stopped by now) a one-shot timer for every subscriber, if it doesn't exist yet
      values->timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(values->timeout_)), [this, key]() {timerCallback(key);});
    }

    if (values->timeout_ > longest_timeout)
    {
      longest_timeout = values->timeout_;
    }
  }

  if (!common_timer_ || longest_timeout != (common_timer_period_ / 2.0))
  {
    // Create another timer for cmd_vel messages from any source, so we can
    // dislodge last active source if it gets stuck without further messages
    common_timer_period_ = longest_timeout * 2.0;
    common_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(common_timer_period_)), std::bind(&CmdVelMux::commonTimerCallback, this));
  }

  RCLCPP_INFO(get_logger(), "CmdVelMux : (re)configured");
}

bool CmdVelMux::addInputToParameterMap(std::map<std::string, ParameterValues> & parsed_parameters, const std::string & input_name, const std::string & input_variable, const rclcpp::Parameter & parameter_value)
{
  if (parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    if (parsed_parameters.count(input_name) > 0)
    {
      parsed_parameters.erase(input_name);
    }
    return true;
  }
  if (parsed_parameters.count(input_name) == 0)
  {
    parsed_parameters.emplace(std::make_pair(input_name, ParameterValues()));
  }

  if (input_variable == "topic")
  {
    if (parameter_value.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
      RCLCPP_WARN(get_logger(), "topic must be a string; ignoring");
      return false;
    }
    parsed_parameters[input_name].topic = parameter_value.as_string();
  }
  else if (input_variable == "timeout")
  {
    if (parameter_value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      RCLCPP_WARN(get_logger(), "timeout must be a double; ignoring");
      return false;
    }
    parsed_parameters[input_name].timeout = parameter_value.as_double();
  }
  else if (input_variable == "priority")
  {
    if (parameter_value.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      RCLCPP_WARN(get_logger(), "priority must be an integer; ignoring");
      return false;
    }

    int64_t priority = parameter_value.as_int();
    if (priority < 0 || priority > std::numeric_limits<uint32_t>::max())
    {
      RCLCPP_WARN(get_logger(), "Priority out of range, must be between 0 and MAX_UINT32");
      return false;
    }
    if (used_priorities_.count(priority) != 0)
    {
      RCLCPP_WARN(get_logger(), "Cannot have duplicate priorities, ignoring");
      return false;
    }
    used_priorities_.insert(priority);
    parsed_parameters[input_name].priority = priority;
  }
  else if (input_variable == "short_desc")
  {
    if (parameter_value.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
      RCLCPP_WARN(get_logger(), "short_desc must be a string; ignoring");
      return false;
    }
    parsed_parameters[input_name].short_desc = parameter_value.as_string();
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Invalid input variable '%s'; ignored", input_variable.c_str());
    return false;
  }

  return true;
}

std::map<std::string, ParameterValues> CmdVelMux::parseFromParametersMap(const std::map<std::string, rclcpp::Parameter> & parameters)
{
  std::map<std::string, ParameterValues> parsed_parameters;
  // Iterate over all parameters and parse their content
  for (const std::pair<std::string, rclcpp::Parameter> & parameter : parameters)
  {
    std::vector<std::string> splits = stringSplit(parameter.first, ".");
    if (splits.size() != 2)
    {
      RCLCPP_WARN(get_logger(), "Invalid or unknown parameter '%s', ignoring", parameter.first.c_str());
      continue;
    }

    if (!addInputToParameterMap(parsed_parameters, splits[0], splits[1], parameter.second))
    {
      parsed_parameters.clear();
      break;
    }
  }

  return parsed_parameters;
}

void CmdVelMux::cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg, std::string key)
{
  // if subscriber was deleted or the one being called right now just ignore
  if (map_.count(key) == 0)
  {
    return;
  }

  // Reset general timer
  common_timer_->reset();

  // Reset timer for this source
  map_[key]->timer_->reset();

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((allowed_ == VACANT) ||
      (allowed_ == key)    ||
      (map_[key]->priority_ > map_[allowed_]->priority_))
  {
    if (allowed_ != key)
    {
      allowed_ = key;

      // Notify the world that a new cmd_vel source took the control
      auto active_msg = std::make_unique<std_msgs::msg::String>();
      active_msg->data = map_[key]->name_;
      active_subscriber_pub_->publish(std::move(active_msg));
    }

    output_topic_pub_->publish(*msg);
  }
}

void CmdVelMux::commonTimerCallback()
{
  if (allowed_ != VACANT)
  {
    // No cmd_vel messages timeout happened for ANYONE, so last active source got stuck without further
    // messages; not a big problem, just dislodge it; but possibly reflect a problem in the controller
    RCLCPP_WARN(get_logger(), "CmdVelMux : No cmd_vel messages from ANY input received in the last %fs", common_timer_period_);
    RCLCPP_WARN(get_logger(), "CmdVelMux : %s dislodged due to general timeout",
                map_[allowed_]->name_.c_str());

    // No cmd_vel messages timeout happened to currently active source, so...
    allowed_ = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    auto active_msg = std::make_unique<std_msgs::msg::String>();
    active_msg->data = "idle";
    active_subscriber_pub_->publish(std::move(active_msg));
  }
}

void CmdVelMux::timerCallback(std::string key)
{
  if (allowed_ == key)
  {
    // No cmd_vel messages timeout happened to currently active source, so...
    allowed_ = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    auto active_msg = std::make_unique<std_msgs::msg::String>();
    active_msg->data = "idle";
    active_subscriber_pub_->publish(std::move(active_msg));
  }
}

rcl_interfaces::msg::SetParametersResult CmdVelMux::parameterUpdate(
  const std::vector<rclcpp::Parameter> & update_parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::map<std::string, rclcpp::Parameter> old_parameters;
  if (!get_parameters("subscribers", old_parameters) || old_parameters.size() <= 1)
  {
    result.successful = false;
    result.reason = "no parameters loaded";
    return result;
  }

  used_priorities_.clear();
  std::map<std::string, ParameterValues> parameters = parseFromParametersMap(old_parameters);

  // And then merge them
  for (const rclcpp::Parameter & parameter : update_parameters)
  {
    std::vector<std::string> splits = stringSplit(parameter.get_name(), ".");
    if (splits.size() != 3)
    {
      RCLCPP_WARN(get_logger(), "Invalid or unknown parameter '%s', ignoring", parameter.get_name().c_str());
      result.successful = false;
      result.reason = "Invalid or unknown parameter";
      break;
    }
    if (splits[0] != "subscribers")
    {
      RCLCPP_WARN(get_logger(), "Unknown parameter prefix '%s', ignoring", parameter.get_name().c_str());
      result.successful = false;
      result.reason = "Unknown parameter prefix";
      break;
    }

    const std::string & input_name = splits[1];
    const std::string & input_variable = splits[2];

    if (!addInputToParameterMap(parameters, input_name, input_variable, parameter))
    {
      result.successful = false;
      result.reason = "Invalid parameter";
      break;
    }
  }

  if (result.successful)
  {
    if (parametersAreValid(parameters))
    {
      configureFromParameters(parameters);
    }
    else
    {
      result.successful = false;
      result.reason = "Incomplete parameters";
    }
  }

  return result;
}

} // namespace cmd_vel_mux

RCLCPP_COMPONENTS_REGISTER_NODE(cmd_vel_mux::CmdVelMux)
