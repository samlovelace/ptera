
#include "PosePublisher.h"
#include <ignition/common/Console.hh>
#include <ignition/math/Quaternion.hh>
#include <thread> 
#include <chrono> 
#include <vector> 


void PosePublisher::Configure(const ignition::gazebo::Entity &entity,
                                         const std::shared_ptr<const sdf::Element> &anSdf,
                                         ignition::gazebo::EntityComponentManager &ecm,
                                         ignition::gazebo::EventManager &)
{
	mRunning = true; 
	mModel = ignition::gazebo::Model(entity);
	if (!mModel.Valid(ecm))
	{
		std::cerr << "invalid model entity." << std::endl;
		return;
	}
	
	std::string publishTopicName = "/robot/state";
	if(anSdf->HasElement("topic_name"))
	{
		publishTopicName = anSdf->Get<std::string>("topic_name"); 
	}

	int rate = 10; 
	if(anSdf->HasElement("rate"))
	{
		rate = anSdf->Get<int>("rate"); 
	}

  	auto ctx = rclcpp::contexts::get_global_default_context(); 
  
	if(!ctx->is_valid())
	{
		rclcpp::init(0, nullptr); 
	} 
	
	mRosNode = rclcpp::Node::make_shared("pose_publisher");
	mPosPub = mRosNode->create_publisher<ptera_msgs::msg::RobotState>(publishTopicName, 10); 

	mRosSpinThread = std::thread([this](){
		rclcpp::spin(mRosNode); 
	}); 

	mPublishRate = std::make_unique<RateController>(rate); 
	
	mPublishThread = std::thread([&](){
		robotStatePublishLoop(); 
	});

	ignmsg << "Configured to publish " << mModel.Name(ecm) << "'s state on " << publishTopicName << " at " << rate << "hz" << std::endl; 

}

void PosePublisher::PostUpdate(const ignition::gazebo::UpdateInfo&, const ignition::gazebo::EntityComponentManager &ecm)
{
    auto pose = ecm.Component<ignition::gazebo::components::Pose>(mModel.Entity()); 

    if(pose)
    {
		ptera_msgs::msg::RobotState idlPose; 
		convertToIdl(pose, idlPose); 
		setLatestState(idlPose); 
    }

}

void PosePublisher::convertToIdl(const ignition::gazebo::components::Pose* aPose, ptera_msgs::msg::RobotState& anIdlPose)
{
	ptera_msgs::msg::Vec3 pos; 
	pos.set__x(aPose->Data().X());
	pos.set__y(aPose->Data().Y()); 
	pos.set__z(aPose->Data().Z()); 

	ptera_msgs::msg::Euler eul; 
	eul.set__pitch(aPose->Data().Pitch()); 
	eul.set__roll(aPose->Data().Roll()); 
	eul.set__yaw(aPose->Data().Yaw()); 

	ignition::math::Quaterniond quat(aPose->Data().Roll(), aPose->Data().Pitch(), aPose->Data().Yaw()); 

	ptera_msgs::msg::Quaternion q; 
	q.set__w(quat.W()); 
	q.set__x(quat.X()); 
	q.set__y(quat.Y()); 
	q.set__z(quat.Z()); 

	// TODO: compute velocities and populate into message 

	auto now = mRosNode->now(); 
	builtin_interfaces::msg::Time nowTime; 
	nowTime.set__nanosec(now.nanoseconds()); 
	nowTime.set__sec(now.seconds()); 

	anIdlPose.set__position(pos); 
	anIdlPose.set__euler(eul); 
	anIdlPose.set__quat(q); 
	anIdlPose.set__timestamp(now);
}

void PosePublisher::robotStatePublishLoop()
{
	while(isRunning())
	{
		mPublishRate->start(); 
		ptera_msgs::msg::RobotState state = getLatestState(); 
		mPosPub->publish(state); 
		mPublishRate->block(); 
	}

}

PosePublisher::~PosePublisher()
{
	setRunning(false); 
	if(mRosSpinThread.joinable())
	{
		mRosSpinThread.join(); 
	}

	if(mPublishThread.joinable())
	{
		mPublishThread.join(); 
	}

	mRosNode = nullptr; 
	
	if(rclcpp::ok())
	{
		rclcpp::shutdown(); 
	}
	
	while(rclcpp::ok())
	{
		std::cout << "shutting down ROS2 plugin" << std::endl; 
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));  
	}
}
