#ifndef _BEHAVIOR_TREE__PLUGINS__IS_HOME_H
#define _BEHAVIOR_TREE__PLUGINS__IS_HOME_H

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <deque>
#include <behaviortree_cpp_v3/condition_node.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "utils/bt_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace autoware_behavior_tree
{
    class IsHomeCondition : public BT::ConditionNode
    {
        bt_utils::BTUtils* utils;

    public:
        IsHomeCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);
        
        IsHomeCondition() = delete;
        ~IsHomeCondition() override;
        BT::NodeStatus tick() override;
        void initialize();

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>(
                "localization_topic", std::string("/localization"), "localization topic"),
                BT::InputPort<geometry_msgs::msg::Pose>("home_position"),
                BT::InputPort<double>("check_threshold"),
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        double check_threshold_;
        void localization_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;
        geometry_msgs::msg::Pose home_position_;
        geometry_msgs::msg::Pose current_position_;
        geometry_msgs::msg::PoseStamped goal;
        std::atomic<bool> is_home_;
        std::string localization_topic_;
        bool initialized_;
    };
}
#endif //_BEHAVIOR_TREE__PLUGINS__IS_HOME_H 