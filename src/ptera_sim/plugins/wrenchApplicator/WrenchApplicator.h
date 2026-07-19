#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <thread>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <chrono>

class WrenchApplicator
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
	WrenchApplicator() = default;

	void Configure(const ignition::gazebo::Entity &entity,
					const std::shared_ptr<const sdf::Element> &,
					ignition::gazebo::EntityComponentManager &ecm,
					ignition::gazebo::EventManager &) override;

	void PreUpdate(const ignition::gazebo::UpdateInfo &,
					ignition::gazebo::EntityComponentManager &ecm) override;

	~WrenchApplicator() override;

private:
	struct CachedWrench
	{
		ignition::math::Vector3d force;
		ignition::math::Vector3d torque;
		std::chrono::steady_clock::time_point receivedAt;
	};

	void wrenchCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg);

	std::unordered_map<std::string, CachedWrench> getLatestWrenches();

	ignition::gazebo::Model mModel{ignition::gazebo::kNullEntity};
	std::shared_ptr<rclcpp::Node> mRosNode;
	rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr mWrenchSub;
	std::thread mRosSpinThread;

	bool mWorldFrame{false};
	double mTimeout{0.5};

	std::mutex mWrenchMutex;
	std::unordered_map<std::string, CachedWrench> mLatestWrenches;

	std::unordered_map<std::string, ignition::gazebo::Link> mLinks;
};

// Plugin registration
IGNITION_ADD_PLUGIN(
    WrenchApplicator,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate)
