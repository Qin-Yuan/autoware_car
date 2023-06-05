#ifndef SOMO_BEHAVIOR_TREE__PLUGINS__ACTION__QUEUE_PRODUCER_CONSUMER_SERVICE_HPP_
#define SOMO_BEHAVIOR_TREE__PLUGINS__ACTION__QUEUE_PRODUCER_CONSUMER_SERVICE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/actions/pop_from_queue.hpp"
#include "behaviortree_cpp_v3/decorators/consume_queue.h"
#include <behaviortree_cpp_v3/condition_node.h>
#include "nav2_behavior_tree/bt_service_node.hpp"

#include <string>
#include "autoware_msgs/srv/param_request.hpp"

namespace autoware_behavior_tree
{  
    class QueueConsumerService : public BT::SyncActionNode
    {
        public:
            QueueConsumerService(
                const std::string & action_name,
                const BT::NodeConfiguration & conf);
            BT::NodeStatus tick() override ;
            void initialize();
            static BT::PortsList providedPorts()
            {
                return {
                    // BT::InputPort<std::string>("QueueName", "生产者生产的Queue名称 {Queue}")
                };
            }
        private:
           rclcpp::Node::SharedPtr node_ ;
           geometry_msgs::msg::PoseStamped goal_;
           bool initialized_ ;
    };

}   // namespace autoware_behavior_tree

#endif  // SOMO_BEHAVIOR_TREE__PLUGINS__ACTION__QUEUE_PRODUCER_CONSUMER_ACTION_HPP_
