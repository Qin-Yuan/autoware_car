#include <string>
#include "autoware_behavior_tree/plugins/condition/is_say_condition.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "autoware_msgs/msg/bt_tts.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
using std::placeholders::_1;

namespace autoware_behavior_tree
{
    IsSayCondition::IsSayCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
        tts_type_("tts"),
        tts_cmd_(""),
        tts_topic_("/tts_topic"),
        IsWaitEnd_("false")
        {   
            EndFlag_ = false ; 
            initialized_ = false ;
            haspub = false ;
            getInput("tts_type", tts_type_);
            getInput("tts_cmd", tts_cmd_);
            getInput("tts_topic", tts_topic_);
            getInput("IsWaitEnd", IsWaitEnd_);

            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            config().blackboard->set<bool>("TtsEndFlag",false);
            config().blackboard->set<bool>("haspub",false);
        }

    IsSayCondition::~IsSayCondition()
    {
        RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsSayCondition BT node");
    }

    BT::NodeStatus IsSayCondition::tick()
    {
        if(!initialized_) {
            initialize();
        }
        
        getInput("tts_type", tts_type_);
        getInput("tts_cmd", tts_cmd_);
        getInput("IsWaitEnd", IsWaitEnd_);
        getInput("tts_topic", tts_topic_);
        
        if(haspub == false) {
            BtTts_.type = tts_type_ ;
            BtTts_.cmd = tts_cmd_ ;
            ttsPub_->publish(BtTts_);
            haspub = true ;
            config().blackboard->set<bool>("haspub",haspub);
            RCLCPP_INFO(node_->get_logger(), "********* Waiting for TTS EndFlag_ ...... *********");
        }
        
        if(IsWaitEnd_ == "true") {
            config().blackboard->get("TtsEndFlag",EndFlag_);
            if ( EndFlag_ == true ) {
                config().blackboard->set("TtsEndFlag",false);
                haspub = false ;
                config().blackboard->set<bool>("haspub",haspub);
                return BT::NodeStatus::SUCCESS;
            }
            else {
                // RCLCPP_INFO(node_->get_logger(), "tts is not end , waiting ......");
                return BT::NodeStatus::RUNNING; 
            }
        }
        haspub = false ;
        return BT::NodeStatus::SUCCESS;
    }

    void IsSayCondition::initialize()
    {   
        getInput("tts_topic", tts_topic_);
        ttsPub_ = node_->create_publisher<autoware_msgs::msg::BtTts>(tts_topic_, 10);
        initialized_ = true;
    }
}


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) 
{
    factory.registerNodeType<autoware_behavior_tree::IsSayCondition>("IsSayCondition");
}
