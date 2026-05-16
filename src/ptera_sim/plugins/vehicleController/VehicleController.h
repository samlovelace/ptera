#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/transport11/ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>

#include <rclcpp/rclcpp.hpp>
#include "ptera_msgs/msg/robot_state.hpp"
#include "ptera_msgs/msg/vehicle_waypoint.hpp"

#include <thread>
#include <memory> 
#include <mutex> 
#include "RateController.h"

class VehicleController
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
	VehicleController() = default;

	void Configure(const ignition::gazebo::Entity &entity,
					const std::shared_ptr<const sdf::Element> &,
					ignition::gazebo::EntityComponentManager &ecm,
					ignition::gazebo::EventManager &) override;

	void PreUpdate(const ignition::gazebo::UpdateInfo &,
					ignition::gazebo::EntityComponentManager &ecm) override;

	~VehicleController() override;

private:
	void commandCallback(ptera_msgs::msg::VehicleWaypoint::SharedPtr msg);
    void navCallback(ptera_msgs::msg::RobotState::SharedPtr msg); 
    void controlLoop(); 

	void setRunning(bool aFlag) {std::lock_guard<std::mutex> lock(mRunMutex); mRunning = aFlag; }
	bool isRunning() {std::lock_guard<std::mutex> lock(mRunMutex); return mRunning; }

    void setLatestNav(const ptera_msgs::msg::RobotState::SharedPtr aNavState); 
    void setLatestCmd(const ptera_msgs::msg::VehicleWaypoint::SharedPtr aWaypoint); 

    ptera_msgs::msg::RobotState::SharedPtr getLatestNav(); 
    ptera_msgs::msg::VehicleWaypoint::SharedPtr getLatestCmd(); 

	void publishTwistCmd(double x, double y, double h);

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
	rclcpp::Subscription<ptera_msgs::msg::VehicleWaypoint>::SharedPtr mCmdSub;
    rclcpp::Subscription<ptera_msgs::msg::RobotState>::SharedPtr mNavSub; 
	 
	std::thread mRosSpinThread;
	std::thread mPublishThread; 
	std::unique_ptr<RateController> mControlRate; 

	std::vector<double> mKp; 
	std::vector<double> mKd; 
	std::vector<double> mPrevPosErr;
	std::chrono::time_point<std::chrono::steady_clock> mPrevTime;  

	std::mutex mCmdMutex;
    ptera_msgs::msg::VehicleWaypoint::SharedPtr mCmd; 
    std::mutex mNavMutex; 
    ptera_msgs::msg::RobotState::SharedPtr mNav; 

    bool mCmdRecvd; 
    bool mNavRecvd;
    bool mControlLoopLaunched;  

    std::thread mControlThread; 

	std::mutex mRunMutex; 
	bool mRunning; 

	ignition::transport::Node mNode;
	ignition::transport::Node::Publisher mCmdVelPub;

};

// Plugin registration
IGNITION_ADD_PLUGIN(
    VehicleController,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)
