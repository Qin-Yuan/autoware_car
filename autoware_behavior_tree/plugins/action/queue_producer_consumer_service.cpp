#include <memory>
#include <string>
#include "autoware_behavior_tree/plugins/action/queue_producer_consumer_service.hpp"

namespace autoware_behavior_tree
{

    QueueConsumerService::QueueConsumerService(
        const std::string & service_node_name,
        const  BT::NodeConfiguration & conf)
        : BT::SyncActionNode(service_node_name, conf)
        {
            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            config().blackboard->set<std::string>("Introduction_goal" , "NULL");
        }

    BT::NodeStatus QueueConsumerService::tick() {
        if(!initialized_) {
            initialize();
        }
        std::shared_ptr<BT::ProtectedQueue<geometry_msgs::msg::PoseStamped>> goal_queue;
        config().blackboard->get("array_goal_pose", goal_queue);
        if (goal_queue) {
            auto& goals = goal_queue->items;
            if (goals.empty()) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "there is not goal !");
                return BT::NodeStatus::SUCCESS;
            }
            goal_ = goals.front();
            goals.pop_front();
            config().blackboard->set("Introduction_goal", goal_.header.frame_id);
            goal_.header.frame_id = "map" ;
            config().blackboard->set("goal", goal_);
            return BT::NodeStatus::SUCCESS;
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "there is not goal !");
            return BT::NodeStatus::SUCCESS;
        }
    }

    void QueueConsumerService::initialize() {
        initialized_ = true ;
    }
}    

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autoware_behavior_tree::QueueConsumerService>("QueueConsumerService");
}
