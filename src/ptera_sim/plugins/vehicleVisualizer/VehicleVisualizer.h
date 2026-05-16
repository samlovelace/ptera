#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/transport11/ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include "ptera_msgs/msg/robot_state.hpp"

#include <thread>
#include <memory> 
#include <mutex> 
#include "RateController.h"

class VehicleVisualizer
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
	VehicleVisualizer() = default;

	void Configure(const ignition::gazebo::Entity &entity,
					const std::shared_ptr<const sdf::Element> &,
					ignition::gazebo::EntityComponentManager &ecm,
					ignition::gazebo::EventManager &) override;

	void PreUpdate(const ignition::gazebo::UpdateInfo &,
					ignition::gazebo::EntityComponentManager &ecm) override;

	~VehicleVisualizer() override;

private:
    void navCallback(ptera_msgs::msg::RobotState::SharedPtr msg); 
    void setLatestNav(const ptera_msgs::msg::RobotState::SharedPtr aNavState); 
    ptera_msgs::msg::RobotState::SharedPtr getLatestNav(); 


  	template <typename T>
	std::vector<T> parseVector(const std::string &str)
	{
		std::vector<T> result;
		std::istringstream iss(str);
		T value;
		while (iss >> value)
		{
			result.push_back(value);
		}
		return result;
	}

	ignition::gazebo::Model mModel{ignition::gazebo::kNullEntity};
	std::shared_ptr<rclcpp::Node> mRosNode;
    rclcpp::Subscription<ptera_msgs::msg::RobotState>::SharedPtr mNavSub; 
	 
	std::thread mRosSpinThread;

    std::mutex mNavMutex; 
    ptera_msgs::msg::RobotState::SharedPtr mNav; 

    bool mNavRecvd;

	ignition::transport::Node mNode;
	ignition::transport::Node::Publisher mCmdVelPub;

};

// Plugin registration
IGNITION_ADD_PLUGIN(
    VehicleVisualizer,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)
