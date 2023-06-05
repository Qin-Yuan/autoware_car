#include <string>
#include "autoware_behavior_tree/plugins/condition/is_home_condition.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


namespace autoware_behavior_tree
{
    IsHomeCondition::IsHomeCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        is_home_(false),
        localization_topic_("/localization"),
        initialized_(false)
        {
          getInput("localization_topic", localization_topic_);
          getInput("home_position", home_position_);
          getInput("check_threshold", check_threshold_);
          node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
          localization_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            localization_topic_,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsHomeCondition::localization_cb, this, std::placeholders::_1));
          config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal" , goal);
          config().blackboard->set<std::string>("go_home_state" , "NULL");
        }
    IsHomeCondition::~IsHomeCondition()
    {
        RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsHomeCondition BT node");
    }

    BT::NodeStatus IsHomeCondition::tick()
    {
      std::cout << initialized_ << std::endl;
      if(!initialized_)
      {
        initialize();
      }
      getInput("home_position", home_position_);
      rclcpp::spin_some(node_);
      geometry_msgs::msg::PoseStamped goal;
      goal.pose.position.x = home_position_.position.x;
      goal.pose.position.y = home_position_.position.y;
      std::cout << "current_position_: " << current_position_.position.x << "," << current_position_.position.y << std::endl;
      std::cout << "home_position_: " << home_position_.position.x << "," << home_position_.position.y << std::endl;
      std::cout << "check_threshold_: " << check_threshold_ << std::endl;
      is_home_ = utils->check_distance(current_position_,home_position_,check_threshold_);
      if (is_home_) {
        std::cout << "SOMO is in home"<< std::endl;
        return BT::NodeStatus::SUCCESS;
      }   
      std::cout << "SOMO is in not home"<< std::endl;
      config().blackboard->set("goal", goal);
      std::string _go_home_state = "我的任务已完成，我要回去了,有什么需要再叫我";
      config().blackboard->set("go_home_state", _go_home_state);

      return BT::NodeStatus::FAILURE; 
    }

    void IsHomeCondition::initialize()
    {
      initialized_ = true;
    }

    void IsHomeCondition::localization_cb(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
      current_position_ = msg->pose.pose;
    }

}


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<autoware_behavior_tree::IsHomeCondition>("IsHomeCondition");
}