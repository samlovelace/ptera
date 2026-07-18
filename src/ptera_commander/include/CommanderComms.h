#ifndef COMMANDERCOMMS_H
#define COMMANDERCOMMS_H

#include "ptera_msgs/msg/joint_position_waypoint.hpp"
#include "ptera_msgs/msg/enable.hpp"
#include "ptera_msgs/msg/task_position_waypoint.hpp"
#include "ptera_msgs/msg/task_velocity_waypoint.hpp"
#include "ptera_msgs/msg/joint_velocity_waypoint.hpp"
#include "ptera_msgs/msg/plan_command.hpp"
#include "ptera_msgs/msg/vision_command.hpp"
#include "ptera_msgs/msg/gpc_goal.hpp"

#include "ptera_msgs/msg/manipulation_command.hpp"
 
class CommanderComms 
{ 
public:
    CommanderComms();
    ~CommanderComms();

    bool start(); 
    bool stop(); 

private:
   
};
#endif //COMMANDERCOMMS_H   