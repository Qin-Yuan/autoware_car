#ifndef _BEHAVIOR_TREE__PLUGINS__IS_SAY_H
#define _BEHAVIOR_TREE__PLUGINS__IS_SAY_H

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <deque>
#include <behaviortree_cpp_v3/condition_node.h>
#include "utils/bt_utils.hpp"
#include "autoware_msgs/msg/bt_tts.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"


namespace autoware_behavior_tree
{  
    class IsSayCondition : public BT::ConditionNode
    {
        bt_utils::BTUtils* utils;
    
    public:
        IsSayCondition(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);
        
        IsSayCondition() = delete;
        ~IsSayCondition() override;
        BT::NodeStatus tick() override;
        // BT::NodeStatus tts_end_tick() override;
        void initialize();

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("tts_type", std::string("tts"), "tts播放功能 tts/introduction"),
                BT::InputPort<std::string>("tts_cmd", std::string("error"), "tts文本信息"),
                BT::InputPort<std::string>("tts_topic", std::string("/tts_topic"), "tts语音话题"),
                BT::InputPort<std::string>("IsWaitEnd", std::string("false"), "tts是否等待结束标志位")
            };
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<autoware_msgs::msg::BtTts>::SharedPtr ttsPub_;
        autoware_msgs::msg::BtTts BtTts_;
        std::string tts_type_;
        std::string tts_cmd_;
        std::string tts_topic_;
        std::string IsWaitEnd_;
        bool initialized_;
        bool EndFlag_ ;
        bool haspub ;
    };
}
#endif //_BEHAVIOR_TREE__PLUGINS__IS_SAY_H 
