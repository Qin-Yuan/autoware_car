#include <string>
#include "autoware_behavior_tree/plugins/condition/has_mission_condition.hpp"

namespace autoware_behavior_tree
{
    HasMissionCondition::HasMissionCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        initialized_(false),
        mission_count(false) 
        {
            setOutput("mission_number",mission_count);
            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        }
        
    HasMissionCondition::~HasMissionCondition(){
        std::cout << "Shutting down IsStuckCondition BT node"<< std::endl;
    }

    BT::NodeStatus HasMissionCondition::tick()
    {   
        // RCLCPP_WARN(node_->get_logger(), "********** mission_number ***********");
        // std::cout << initialized_ << std::endl;
        if(!initialized_)
        {
            initialize();
        }
        config().blackboard->get("mission_number", mission_count);
        // std::cout << "get mission number" << std::endl;
        // cant't show it on Groot?
        setOutput("mission_number",mission_count);
        // std::cout << "mission number is " << mission_count << std::endl;
        return mission_count ? BT::NodeStatus::SUCCESS: BT::NodeStatus::FAILURE;
    }

    void HasMissionCondition::initialize()
    {
        initialized_ = true;
    }

}


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autoware_behavior_tree::HasMissionCondition>("HasMissionCondition");
}