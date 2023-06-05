/**
 * @file /include/yocs_cmd_vel_mux/cmd_vel_mux_nodelet.hpp
 *
 * @brief Structure for the yocs_cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef CMD_VEL_MUX__CMD_VEL_MUX_HPP_
#define CMD_VEL_MUX__CMD_VEL_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{

/*****************************************************************************
 ** CmdVelMux
 *****************************************************************************/

 struct ParameterValues
 {
   std::string topic;
   double timeout{-1.0};
   int64_t priority{-1};
   std::string short_desc;
 };

class CmdVelMux final : public rclcpp::Node
{
public:
  explicit CmdVelMux(rclcpp::NodeOptions options);
  ~CmdVelMux() override = default;
  CmdVelMux(CmdVelMux && c) = delete;
  CmdVelMux & operator=(CmdVelMux && c) = delete;
  CmdVelMux(const CmdVelMux & c) = delete;
  CmdVelMux & operator=(const CmdVelMux & c) = delete;

private:
  static const std::string VACANT;  /**< ID for "nobody" active input;*/

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_topic_pub_;   /**< Multiplexed command velocity topic */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_subscriber_pub_;  /**< Currently allowed cmd_vel subscriber */
  rclcpp::TimerBase::SharedPtr common_timer_;           /**< No messages from any subscriber timeout */
  double common_timer_period_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::string allowed_;

  void commonTimerCallback();
  void timerCallback(std::string key);
  void cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg, std::string key);

  rcl_interfaces::msg::SetParametersResult parameterUpdate(
    const std::vector<rclcpp::Parameter> & update_parameters);

  /*********************
   ** Private Classes
   **********************/

  /**
   * Inner class describing an individual subscriber to a cmd_vel topic
   */
  struct CmdVelSub final
  {
    std::string            name_;         /**< Descriptive name; must be unique to this subscriber */
    std::string            topic_;        /**< The name of the topic */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        sub_;         /**< The subscriber itself */
    rclcpp::TimerBase::SharedPtr             timer_;        /**< No incoming messages timeout */
    double                 timeout_;      /**< Timer's timeout, in seconds  */
    uint32_t               priority_;     /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string            short_desc_;   /**< Short description (optional) */
  };

  bool addInputToParameterMap(std::map<std::string, ParameterValues> & parsed_parameters, const std::string & input_name, const std::string & input_variable, const rclcpp::Parameter & parameter_value);
  bool parametersAreValid(const std::map<std::string, ParameterValues> & parameters);
  void configureFromParameters(const std::map<std::string, ParameterValues> & parameters);
  std::map<std::string, ParameterValues> parseFromParametersMap(const std::map<std::string, rclcpp::Parameter> & parameters);

  std::map<std::string, std::shared_ptr<CmdVelSub>> map_;
  std::set<unsigned int> used_priorities_;
};

} // namespace cmd_vel_mux

#endif /* CMD_VEL_MUX__CMD_VEL_MUX_HPP_ */
