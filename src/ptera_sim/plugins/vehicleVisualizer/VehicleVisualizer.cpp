
#include "VehicleVisualizer.h"
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>

void VehicleVisualizer::Configure(const ignition::gazebo::Entity &entity,
    const std::shared_ptr<const sdf::Element> &anSdf,
    ignition::gazebo::EntityComponentManager &ecm,
    ignition::gazebo::EventManager &)
{
    mNavRecvd = false;
    mModel = ignition::gazebo::Model(entity);
    if (!mModel.Valid(ecm))
    {
        std::cerr << "Invalid model entity" << std::endl;
        return;
    }

    // Request WorldPose to be tracked on the canonical link
    auto linkEntity = mModel.LinkByName(ecm, "base_link");
    if (linkEntity != ignition::gazebo::kNullEntity)
    {
        ignition::gazebo::enableComponent<ignition::gazebo::components::WorldPose>(ecm, linkEntity);
    }
    else
    {
        std::cerr << "[VehicleVisualizer] base_link not found in Configure" << std::endl;
    }

    // Try the model entity instead of the link
    auto* worldPoseCmd = ecm.Component<ignition::gazebo::components::WorldPoseCmd>(entity);
    std::cout << "[VehicleVisualizer] WorldPoseCmd on MODEL entity: " << worldPoseCmd << std::endl;

    auto* pose = ecm.Component<ignition::gazebo::components::Pose>(entity);
    std::cout << "[VehicleVisualizer] Pose on MODEL entity: " << pose << std::endl;

    if (linkEntity != ignition::gazebo::kNullEntity)
    {
        auto* linkPoseCmd = ecm.Component<ignition::gazebo::components::WorldPoseCmd>(linkEntity);
        std::cout << "[VehicleVisualizer] WorldPoseCmd on LINK entity: " << linkPoseCmd << std::endl;

        auto* linkPose = ecm.Component<ignition::gazebo::components::Pose>(linkEntity);
        std::cout << "[VehicleVisualizer] Pose on LINK entity: " << linkPose << std::endl;
    }

    auto ctx = rclcpp::contexts::get_global_default_context();
    if (!ctx->is_valid())
        rclcpp::init(0, nullptr);

    mRosNode = rclcpp::Node::make_shared("vehicle_visualizer");
    mNavSub = mRosNode->create_subscription<ptera_msgs::msg::RobotState>("/robot/state", 10,
        std::bind(&VehicleVisualizer::navCallback, this, std::placeholders::_1));

    mRosSpinThread = std::thread([this](){
        rclcpp::spin(mRosNode);
    });
}

void VehicleVisualizer::PreUpdate(const ignition::gazebo::UpdateInfo&,
                                   ignition::gazebo::EntityComponentManager &ecm)
{
    if (!mNavRecvd)
        return;

    auto nav = getLatestNav();
    if (!nav)
        return;

    ignition::math::Pose3d pose(
        ignition::math::Vector3d(nav->position.x, nav->position.y, 0),
        ignition::math::Quaterniond(nav->quat.w, nav->quat.x, nav->quat.y, nav->quat.z)
    );

    // Write to Pose on the model entity directly
    auto* modelPose = ecm.Component<ignition::gazebo::components::Pose>(mModel.Entity());
    if (modelPose)
    {
        modelPose->Data() = pose;
        ecm.SetChanged(mModel.Entity(),
            ignition::gazebo::components::Pose::typeId,
            ignition::gazebo::ComponentState::OneTimeChange);
    }
}

void VehicleVisualizer::setLatestNav(const ptera_msgs::msg::RobotState::SharedPtr aNavState)
{
    std::lock_guard<std::mutex> lock(mNavMutex); 
    mNav = aNavState; 
}

ptera_msgs::msg::RobotState::SharedPtr VehicleVisualizer::getLatestNav()
{
    std::lock_guard<std::mutex> lock(mNavMutex); 
    return mNav; 
} 

void VehicleVisualizer::navCallback(ptera_msgs::msg::RobotState::SharedPtr aMsg)
{
    if(!mNavRecvd)
    {
        std::cout << "$$$$$$$$$$$$$$$$$$ GOT NAV $$$$$$$$$$$$$$$$$$$" << std::endl; 
        mNavRecvd = true; 
    }

    setLatestNav(aMsg); 
}

VehicleVisualizer::~VehicleVisualizer()
{
    if(mRosSpinThread.joinable())
    {
        mRosSpinThread.join();
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
