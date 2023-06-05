#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "autoware_behavior_tree/bt_task_manager.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


namespace autoware_behavior_tree
{
BtTaskManager::BtTaskManager(const rclcpp::NodeOptions & options)
:   nav2_util::LifecycleNode("bt_task_manager", "", options)
{
    RCLCPP_INFO(get_logger(), "Creating");
    logger_ = get_logger();
    clock_ = get_clock();

    const std::vector<std::string> plugin_libs = {
        "queue_producer_consumer_service_bt_node",
        "has_mission_condition_bt_node", 
        "is_home_condition_bt_node",
        "is_say_condition_bt_node",
        "nav2_compute_path_to_pose_action_bt_node",
        "nav2_compute_path_through_poses_action_bt_node",
        "nav2_smooth_path_action_bt_node",
        "nav2_follow_path_action_bt_node",
        "nav2_spin_action_bt_node",
        "nav2_wait_action_bt_node",
        "nav2_assisted_teleop_action_bt_node",
        "nav2_back_up_action_bt_node",
        "nav2_drive_on_heading_bt_node",
        "nav2_clear_costmap_service_bt_node",
        "nav2_is_stuck_condition_bt_node",
        "nav2_goal_reached_condition_bt_node",
        "nav2_initial_pose_received_condition_bt_node",
        "nav2_goal_updated_condition_bt_node",
        "nav2_globally_updated_goal_condition_bt_node",
        "nav2_is_path_valid_condition_bt_node",
        "nav2_reinitialize_global_localization_service_bt_node",
        "nav2_rate_controller_bt_node",
        "nav2_distance_controller_bt_node",
        "nav2_speed_controller_bt_node",
        "nav2_truncate_path_action_bt_node",
        "nav2_truncate_path_local_action_bt_node",
        "nav2_goal_updater_node_bt_node",
        "nav2_recovery_node_bt_node",
        "nav2_pipeline_sequence_bt_node",
        "nav2_round_robin_node_bt_node",
        "nav2_transform_available_condition_bt_node",
        "nav2_time_expired_condition_bt_node",
        "nav2_path_expiring_timer_condition",
        "nav2_distance_traveled_condition_bt_node",
        "nav2_single_trigger_bt_node",
        "nav2_goal_updated_controller_bt_node",
        "nav2_is_battery_low_condition_bt_node",
        "nav2_navigate_through_poses_action_bt_node",
        "nav2_navigate_to_pose_action_bt_node",
        "nav2_remove_passed_goals_action_bt_node",
        "nav2_planner_selector_bt_node",
        "nav2_controller_selector_bt_node",
        "nav2_goal_checker_selector_bt_node",
        "nav2_controller_cancel_bt_node",
        "nav2_path_longer_on_approach_bt_node",
        "nav2_wait_cancel_bt_node",
        "nav2_spin_cancel_bt_node",
        "nav2_assisted_teleop_cancel_bt_node",
        "nav2_back_up_cancel_bt_node",
        "nav2_drive_on_heading_cancel_bt_node"
    };

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("autoware_behavior_tree");
    // Declare this node's parameters
    declare_parameter("bt_loop_duration", 10);
    declare_parameter("check_threshold", 0.5);
    declare_parameter("default_server_timeout", 20);
    declare_parameter("enable_groot_monitoring", true);
    declare_parameter("groot_zmq_publisher_port", 1666);
    declare_parameter("groot_zmq_server_port", 1667);
    declare_parameter<std::string>("default_bt_xml_filename",pkg_share_dir +"/behavior_trees/autoware_bt.xml");
    declare_parameter("plugin_lib_names", plugin_libs);
}
BtTaskManager::~BtTaskManager()
{}

nav2_util::CallbackReturn
BtTaskManager::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(),"BT TASK MANAGER IS CONFIGURING!!!");

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);




    // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args",
        "-r", std::string("__node:=") + get_name() + "_rclcpp_node",
        "--"});
    client_node_ = std::make_shared<rclcpp::Node>("_", options);

    action_server_ = std::make_shared<ActionServer>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "control_command_handle", std::bind(&BtTaskManager::controlCommandCallback, this));

    ttsEndSub_ = create_subscription<std_msgs::msg::Bool>(
        "speak_end",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&BtTaskManager::TtsEnd_callback, this, std::placeholders::_1));

    param_client_ = create_client<autoware_msgs::srv::ParamRequest>("param_handler");


    //Get parameters
    check_threshold_ =  get_parameter("check_threshold").as_double();
    plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();

    int timeout;
    get_parameter("bt_loop_duration", timeout);
    bt_loop_duration_ = std::chrono::milliseconds(timeout);
    get_parameter("default_server_timeout", timeout);
    default_server_timeout_ = std::chrono::milliseconds(timeout);

    // Create the class that registers our custom nodes and executes the BT
    bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

    blackboard_ = BT::Blackboard::create();
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT
  // Put items on the blackboard
    blackboard_->set<int>("reset_flag", 0);
    blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
      blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
    blackboard_->set<bool>("path_updated", false);  // NOLINT
    blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);
    blackboard_->set<bool>("initial_pose_received", false);  // NOLINT
    blackboard_->set<int>("number_recoveries", 0);  // NOLINT
    blackboard_->set<bool>("mission_number",false);
    blackboard_->set<std::string>("mission_type", "none");
    blackboard_->set<std::string>("child_type", "none");
    blackboard_->set<std::string>("mission_parameter", "none");
    // geometry_msgs::msg::PoseStamped goal;
    // blackboard_->set<geometry_msgs::msg::PoseStamped>("goal_pose", goal);
    blackboard_->set<double>("check_threshold",check_threshold_);

    // Get the BT filename to use from the node parameter
    get_parameter("default_bt_xml_filename", default_bt_xml_filename_);


    return nav2_util::CallbackReturn::SUCCESS;


}

nav2_util::CallbackReturn
BtTaskManager::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_ERROR(get_logger(), " XML file is: %s", default_bt_xml_filename_.c_str());
    if (!loadBehaviorTree(default_bt_xml_filename_)) {
        RCLCPP_ERROR(get_logger(), "Error loading XML file: %s", default_bt_xml_filename_.c_str());
        return nav2_util::CallbackReturn::FAILURE;
    }

    action_server_->activate();
    // create bond connection
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtTaskManager::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    action_server_->deactivate();
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtTaskManager::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    ttsEndSub_.reset();
    client_node_.reset();
    action_server_.reset();
    topic_logger_.reset();
    plugin_lib_names_.clear();
    current_bt_xml_filename_.clear();
    bt_->resetGrootMonitor();
    blackboard_.reset();
    bt_->haltAllActions(tree_.rootNode());
    bt_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtTaskManager::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}


bool BtTaskManager::loadBehaviorTree(const std::string & bt_xml_filename)
{
    
     // Empty filename is default for backward compatibility
    auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

    // Use previous BT if it is the existing one
    if (current_bt_xml_filename_ == filename) {
        RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
        return true;
    }
    bt_->resetGrootMonitor();


    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(filename);

    if (!xml_file.good()) {
        RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
        return false;
    }

    auto xml_string = std::string(
        std::istreambuf_iterator<char>(xml_file),
        std::istreambuf_iterator<char>());

    // Create the Behavior Tree from the XML input
    try {
        tree_ = bt_->createTreeFromText(xml_string, blackboard_);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Exception when loading BT: %s", e.what());
        return false;
    }

    topic_logger_ = std::make_unique<autoware_behavior_tree::RosTopicLogger>(client_node_, tree_);

    current_bt_xml_filename_ = filename;
    std::cout << "成功加载行为树" << std::endl;

    if (get_parameter("enable_groot_monitoring").as_bool()) {
        uint16_t zmq_publisher_port = get_parameter("groot_zmq_publisher_port").as_int();
        uint16_t zmq_server_port = get_parameter("groot_zmq_server_port").as_int();
        // optionally add max_msg_per_second = 25 (default) here
        try {
        bt_->addGrootMonitoring(&tree_, zmq_publisher_port, zmq_server_port);
        } catch (const std::logic_error & e) {
        RCLCPP_ERROR(get_logger(), "ZMQ already enabled, Error: %s", e.what());
        }
    }
    return true;
}

bool BtTaskManager::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
    std::cout << "进入任行为树切换函数" << std::endl;
    std::cout << goal->behavior_tree.c_str()<< std::endl;
    auto bt_xml_filename = goal->behavior_tree;
    if (!loadBehaviorTree(bt_xml_filename)) {
        RCLCPP_ERROR(
        logger_, "BT file not found: %s. Voice mission canceled.",
        bt_xml_filename.c_str());
        return false;
    }
    return true;

}


void BtTaskManager::controlCommandCallback()
{
    std::cout << "进入任务调度回调函数" << std::endl;
    if(!goalReceived(action_server_->get_current_goal())){
        RCLCPP_ERROR(get_logger(),"CANNOT Load bt xml file, mission failed");
        action_server_->terminate_current();
    }
    std::cout << "成功取到目标点" << std::endl;

    auto is_canceling = [this]() {
        if (action_server_ == nullptr) {
            RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
            return true;
        }

        if (!action_server_->is_server_active()) {
            RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
            return true;
        }

        return action_server_->is_cancel_requested();
        };


    RosTopicLogger topic_logger(client_node_, tree_);
    std::shared_ptr<autoware_msgs::action::ControlCommand::Feedback> feedback_msg = std::make_shared<autoware_msgs::action::ControlCommand::Feedback>();
    
    auto on_loop = [&]() {
        if (action_server_->is_preempt_requested()) {
            RCLCPP_INFO(logger_, "Received goal preemption request");
            action_server_->accept_pending_goal();
            // initializeGoalPose();
        }
        topic_logger.flush();

        // action server feedback (pose, duration of task,
        // number of recoveries, and distance remaining to goal)
        feedback_msg->feedback = "Normal running!";
        action_server_->publish_feedback(feedback_msg);
        };
    std::cout << "\033[31m 经过 on loop\033[0m"<< std::endl;
    bool mission_number = action_server_->get_current_goal()->mission_number;
    std::string mission_type = action_server_->get_current_goal()->mission_type;
    std::string child_type = action_server_->get_current_goal()->child_type;
    std::string mission_parameter = action_server_->get_current_goal()->parameters ;

    std::shared_ptr<BT::ProtectedQueue<geometry_msgs::msg::PoseStamped>> array_goal_pose_;
    int arrary_pose_length_ = 0;
    std::cout << "\033[31m 解析请求\033[0m"<< std::endl;
    blackboard_->set<bool>("mission_number",mission_number);
    blackboard_->set<std::string>("mission_type",mission_type);
    blackboard_->set<std::string>("child_type",child_type);
    blackboard_->set<std::string>("mission_parameter",mission_parameter);
    blackboard_->set<std::shared_ptr<BT::ProtectedQueue<geometry_msgs::msg::PoseStamped>>>("array_goal_pose",array_goal_pose_);
    blackboard_->set<int>("array_pose_length",arrary_pose_length_);
    blackboard_->set<bool>("TtsEndFlag",false);
    std::cout << "\033[31m 设置blackboard\033[0m"<< std::endl;
    handleParameters();
    std::cout << "\033[31m 请求参数服务器完毕 \033[0m"<< std::endl;
    // Execute the BT that was previously created in the configure step
    nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling);
    std::cout << "\033[31m 已经触发BT\033[0m"<< std::endl;
    // Make sure that the Bt is not in a running state from a previous execution
    // note: if all the ControlNodes are implemented correctly, this is not needed.
    bt_->haltAllActions(tree_.rootNode());

    switch (rc) {
        case nav2_behavior_tree::BtStatus::SUCCEEDED:
        RCLCPP_INFO(logger_, "Voice mission succeeded");
        action_server_->succeeded_current();
        break;

        case nav2_behavior_tree::BtStatus::FAILED:
        RCLCPP_ERROR(logger_, "Voice mission failed");
        action_server_->terminate_current();
        break;

        case nav2_behavior_tree::BtStatus::CANCELED:
        RCLCPP_INFO(logger_, "Voice mission canceled");
        action_server_->terminate_all();
        break;
    }
}

void BtTaskManager::handleParameters()
{
    std::string mission_parameter;
    std::string mission_type;
    bool mission_number = true;
    geometry_msgs::msg::PoseStamped goal;
    auto array_goal_pose_ = std::make_shared<BT::ProtectedQueue<geometry_msgs::msg::PoseStamped>>();

    blackboard_->get("mission_type", mission_type); 
    blackboard_->get("mission_parameter", mission_parameter);
    blackboard_->get("mission_number", mission_number);
    if (!mission_number){
        auto req = std::make_shared<autoware_msgs::srv::ParamRequest::Request>();
        req->param = mission_parameter.c_str();
        req->mission_type = mission_type.c_str();
        auto resp = param_client_->async_send_request(req);
        auto result = resp.get() ;
        home_position_.position.x = result->goal.pose.position.x;
        home_position_.position.y = result->goal.pose.position.y;
        blackboard_->set<geometry_msgs::msg::Pose>("home_position",home_position_);
    }else{
        if (mission_type == "introduction"){
        auto req = std::make_shared<autoware_msgs::srv::ParamRequest::Request>();
        req->param = mission_parameter.c_str();
        req->mission_type = mission_type.c_str();
        auto resp = param_client_->async_send_request(req);
        auto result = resp.get() ;
        std::cout << "\033[31m 导航点长度1 : \033[0m" << int(result->goals.size()) <<std::endl;
        if (result->goal.header.frame_id == "error") {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response not avilable !");
        }
        else{
            for(auto s : result->goals){
                array_goal_pose_->items.push_back(s);
                std::cout << "\033[31m 导航点 ： \033[0m" << s.header.frame_id <<std::endl;
            }
            blackboard_->set("array_pose_length",int(result->goals.size()));
            blackboard_->set("array_goal_pose",array_goal_pose_);
        }
        }else if (mission_type == "nav"){
        auto req = std::make_shared<autoware_msgs::srv::ParamRequest::Request>();
        req->param = mission_parameter.c_str();
        req->mission_type = mission_type.c_str();
        auto resp = param_client_->async_send_request(req);
        goal = resp.get()->goal;
        if (goal.header.frame_id == "error") {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response not avilable !");
        }else{
            std::cout << "goal:" << goal.pose.position.x << std::endl;
            blackboard_->set("goal", goal);
        }
        }else if (mission_type == "vision"){
        blackboard_->set("face_id", mission_parameter);
        return;
        }
        }
}

void BtTaskManager::TtsEnd_callback(const std_msgs::msg::Bool::SharedPtr msg) 
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "***************\ntts EndFlag_ is true ! %d" , msg->data);
    static bool haspub = false ;
    blackboard_->get("haspub",haspub);
    if (haspub == true) {
        usleep(1000 * 1000);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "********* TTS EndFlag_ is true...... *********");
        haspub = false ;
        blackboard_->set<bool>("TtsEndFlag",msg->data);
    }
}

void BtTaskManager::resetFlags()
{
    blackboard_->set<int>("reset_flag", 0);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_behavior_tree::BtTaskManager)

