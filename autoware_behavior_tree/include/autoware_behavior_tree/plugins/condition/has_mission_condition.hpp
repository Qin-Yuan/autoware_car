#ifndef _BEHAVIOR_TREE__PLUGINS__HAS_MISSION_H
#define _BEHAVIOR_TREE__PLUGINS__HAS_MISSION_H

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <behaviortree_cpp_v3/condition_node.h>

namespace autoware_behavior_tree
{
    class HasMissionCondition : public BT::ConditionNode
    {
    public:
        HasMissionCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);
        
        HasMissionCondition() = delete;
        ~HasMissionCondition() override;
        BT::NodeStatus tick() override;
        void initialize();

        static BT::PortsList providedPorts()
        {
            return {
                    BT::OutputPort<bool>("mission_number","false/true"),
            };
        }
    private:
        rclcpp::Node::SharedPtr node_;
        bool initialized_;
        bool mission_count;
    };
}
#endif //_BEHAVIOR_TREE__PLUGINS__HAS_MISSION_H