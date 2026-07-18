
#include "CommanderComms.h"
#include "RosTopicManager.hpp"


CommanderComms::CommanderComms()
{
    rclcpp::init(0, nullptr); 
}

CommanderComms::~CommanderComms()
{

}

bool CommanderComms::start()
{

    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createPublisher<ptera_msgs::msg::JointPositionWaypoint>("arm/joint_position_waypoint"); 
    topicManager->createPublisher<ptera_msgs::msg::Enable>("arm/enable"); 
    topicManager->createPublisher<ptera_msgs::msg::TaskPositionWaypoint>("arm/task_position_waypoint");
    topicManager->createPublisher<ptera_msgs::msg::TaskVelocityWaypoint>("arm/task_velocity_waypoint"); 
    topicManager->createPublisher<ptera_msgs::msg::JointVelocityWaypoint>("arm/joint_velocity_waypoint");  
    topicManager->createPublisher<ptera_msgs::msg::PlanCommand>("arm/plan"); // TODO: remove or update 
    topicManager->createPublisher<ptera_msgs::msg::VisionCommand>("vision/command"); 
    topicManager->createPublisher<ptera_msgs::msg::GpcGoal>("gpc/goal"); 
    topicManager->createPublisher<ptera_msgs::msg::ManipulationCommand>("arm/command");
    
    topicManager->spinNode(); 

    return true; 
}

bool CommanderComms::stop()
{
    rclcpp::shutdown(); 
}