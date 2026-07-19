#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Joint.hh>

#include <rclcpp/rclcpp.hpp>
#include "ptera_msgs/msg/robot_state.hpp"

#include <thread>
#include <memory> 
#include <mutex> 
#include "RateController.h"

class PosePublisher
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPostUpdate
{
public:
	PosePublisher() = default;

	void Configure(const ignition::gazebo::Entity &entity,
					const std::shared_ptr<const sdf::Element> &,
					ignition::gazebo::EntityComponentManager &ecm,
					ignition::gazebo::EventManager &) override;

	void PostUpdate(const ignition::gazebo::UpdateInfo &,
					const ignition::gazebo::EntityComponentManager &ecm) override;

	~PosePublisher() override;

private:

	ignition::gazebo::Model mModel{ignition::gazebo::kNullEntity};
	std::shared_ptr<rclcpp::Node> mRosNode;
	rclcpp::Publisher<ptera_msgs::msg::RobotState>::SharedPtr mPosPub; 
	std::thread mRosSpinThread;
	std::thread mPublishThread; 
	std::unique_ptr<RateController> mPublishRate; 

	void setRunning(bool aFlag) {std::lock_guard<std::mutex> lock(mRunMutex); mRunning = aFlag; }
	bool isRunning() {std::lock_guard<std::mutex> lock(mRunMutex); return mRunning; }
	std::mutex mRunMutex; 
	bool mRunning; 

	void robotStatePublishLoop();
	void convertToIdl(const ignition::gazebo::components::Pose* aPose,
					   const ignition::gazebo::EntityComponentManager &ecm,
					   ptera_msgs::msg::RobotState& anIdlPose);

	void setLatestState(ptera_msgs::msg::RobotState aState) {std::lock_guard<std::mutex> lock(mStateMutex); mLatestState = aState;}
	ptera_msgs::msg::RobotState getLatestState() {std::lock_guard<std::mutex> lock(mStateMutex); return mLatestState; }

	std::mutex mStateMutex;
	ptera_msgs::msg::RobotState mLatestState;

	ignition::gazebo::Link mLink{ignition::gazebo::kNullEntity};

};

// Plugin registration
IGNITION_ADD_PLUGIN(
    PosePublisher,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPostUpdate)
