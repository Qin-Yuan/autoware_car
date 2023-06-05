#ifndef SOMO_BT_BEHAVIOR_TREE__BT_TASK_MANAGER_HPP_
#define SOMO_BT_BEHAVIOR_TREE__BT_TASK_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include "autoware_msgs/action/control_command.hpp"
#include "autoware_msgs/srv/param_request.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_behavior_tree/behavior_tree_engine.hpp"
#include "autoware_behavior_tree/ros_topic_logger.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp_v3/decorators/consume_queue.h"

namespace autoware_behavior_tree
{
    using ActionT = autoware_msgs::action::ControlCommand;
class BtTaskManager : public nav2_util::LifecycleNode
{
public:
    using ActionServer = nav2_util::SimpleActionServer<ActionT>;
    typedef std::function<bool (typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
    typedef std::function<void ()> OnLoopCallback;
    typedef std::function<void (typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;
    typedef std::function<void (typename ActionT::Result::SharedPtr,
        nav2_behavior_tree::BtStatus)> OnCompletionCallback;

    explicit BtTaskManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~BtTaskManager();

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;




    bool loadBehaviorTree(const std::string & bt_xml_filename = "");

    BT::Blackboard::Ptr getBlackboard() const
    {
        return blackboard_;
    }

    std::string getCurrentBTFilename() const
    {
        return current_bt_xml_filename_;
    }

    std::string getDefaultBTFilename() const
    {
        return default_bt_xml_filename_;
    }

    const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
    {
        return action_server_->accept_pending_goal();
    }

    void terminatePendingGoal()
    {
        action_server_->terminate_pending_goal();
    }

    const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
    {
        return action_server_->get_current_goal();
    }

    const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const
    {
        return action_server_->get_pending_goal();
    }

    void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
    {
        action_server_->publish_feedback(feedback);
    }

    const BT::Tree & getTree() const
    {
        return tree_;
    }

    void haltTree()
    {
        tree_.rootNode()->halt();
    }


    void controlCommandCallback();

    void handleParameters();

    void TtsEnd_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void resetFlags();

    bool goalReceived(ActionT::Goal::ConstSharedPtr goal); 



protected:

    std::string action_name_;

    // Our action server implements the template action
    std::shared_ptr<ActionServer> action_server_;

    // Behavior Tree to be executed when goal is received
    BT::Tree tree_;

    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;

    // The XML file that cointains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    // The wrapper class for the BT functionality
    std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

    // Libraries to pull plugins (BT Nodes) from
    std::vector<std::string> plugin_lib_names_;

    // A regular, non-spinning ROS node that we can use for calls to the action client
    rclcpp::Node::SharedPtr client_node_;

    // Parent node
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    // Clock
    rclcpp::Clock::SharedPtr clock_;

    // Logger
    rclcpp::Logger logger_{rclcpp::get_logger("BtTaskManager")};

    // To publish BT logs
    std::unique_ptr<autoware_behavior_tree::RosTopicLogger> topic_logger_;

    // Duration for each iteration of BT execution
    std::chrono::milliseconds bt_loop_duration_;

    // Default timeout value while waiting for response from a server
    std::chrono::milliseconds default_server_timeout_;

    // User-provided callbacks
    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;

    rclcpp::Client<autoware_msgs::srv::ParamRequest>::SharedPtr param_client_;
    double check_threshold_;
    geometry_msgs::msg::Pose home_position_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ttsEndSub_;

      // Spinning transform that can be used by the BT nodes
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;



};

}
#endif  // SOMO_BT_BEHAVIOR_TREE__BT_TASK_MANAGER_HPP_