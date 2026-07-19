#include "WrenchApplicator.h"
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Link.hh>
#include "LinkResolver.h"

void WrenchApplicator::Configure(const ignition::gazebo::Entity &entity,
                                  const std::shared_ptr<const sdf::Element> &anSdf,
                                  ignition::gazebo::EntityComponentManager &ecm,
                                  ignition::gazebo::EventManager &)
{
  mModel = ignition::gazebo::Model(entity);
  if (!mModel.Valid(ecm))
  {
    std::cerr << "invalid model entity." << std::endl;
    return;
  }

  std::string topicName = "wrench_cmd";
  if (anSdf && anSdf->HasElement("topic"))
  {
    topicName = anSdf->Get<std::string>("topic");
  }

  std::string frame = "body";
  if (anSdf && anSdf->HasElement("frame"))
  {
    frame = anSdf->Get<std::string>("frame");
  }
  mWorldFrame = (frame == "world");

  if (anSdf && anSdf->HasElement("timeout"))
  {
    mTimeout = anSdf->Get<double>("timeout");
  }

  ignmsg << "WrenchApplicator subscribing to " << topicName
         << " (frame=" << frame << ", timeout=" << mTimeout << "s)\n";

  auto ctx = rclcpp::contexts::get_global_default_context();

  if (!ctx->is_valid())
  {
    rclcpp::init(0, nullptr);
  }

  mRosNode = rclcpp::Node::make_shared("wrench_applicator");
  mWrenchSub = mRosNode->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topicName, 10, std::bind(&WrenchApplicator::wrenchCallback, this, std::placeholders::_1));

  mRosSpinThread = std::thread([this]() {
    rclcpp::spin(mRosNode);
  });
}

void WrenchApplicator::PreUpdate(const ignition::gazebo::UpdateInfo &, ignition::gazebo::EntityComponentManager &ecm)
{
  if (!mModel.Valid(ecm))
  {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  auto wrenches = getLatestWrenches();

  for (const auto &entry : wrenches)
  {
    const std::string &linkName = entry.first;
    const CachedWrench &wrench = entry.second;

    double age = std::chrono::duration<double>(now - wrench.receivedAt).count();
    if (age > mTimeout)
    {
      ignmsg << "Wrench for link '" << linkName << "' is too old (age=" << age << "s, timeout=" << mTimeout << "s)\n";
      continue;
    }

    auto it = mLinks.find(linkName);
    if (it == mLinks.end())
    {
      ignition::gazebo::Entity linkEntity = resolveLinkEntity(mModel.Entity(), ecm, linkName);

      if (linkEntity == ignition::gazebo::kNullEntity)
      {
        ignerr << "WrenchApplicator: link '" << linkName << "' not found on model '"
               << mModel.Name(ecm) << "'\n";
        mLinks.emplace(linkName, ignition::gazebo::Link(ignition::gazebo::kNullEntity));
        continue;
      }

      ignition::gazebo::Link link(linkEntity);
      if (!mWorldFrame)
      {
        ignition::gazebo::enableComponent<ignition::gazebo::components::WorldPose>(ecm, linkEntity);
      }

      it = mLinks.emplace(linkName, link).first;
    }

    ignition::gazebo::Link &link = it->second;
    if (link.Entity() == ignition::gazebo::kNullEntity)
    {
      continue;
    }

    ignition::math::Vector3d forceWorld = wrench.force;
    ignition::math::Vector3d torqueWorld = wrench.torque;

    if (!mWorldFrame)
    {
      auto pose = link.WorldPose(ecm);
      if (!pose.has_value())
      {
        continue;
      }

      forceWorld = pose->Rot().RotateVector(wrench.force);
      torqueWorld = pose->Rot().RotateVector(wrench.torque);
    }

    // Apply the force at the link's center of mass rather than its origin -
    // AddWorldForce's single-vector overload looks up the link's inertial
    // offset and cancels the induced moment internally, so a force through
    // the CoM produces pure translation with no coupled rotation. The
    // torque is then added separately as a pure couple (zero force here
    // means it can't pick up any position-dependent moment of its own), so
    // force and torque act independently instead of contaminating each
    // other.
    link.AddWorldForce(ecm, forceWorld);
    link.AddWorldWrench(ecm, ignition::math::Vector3d::Zero, torqueWorld);
  }
}

void WrenchApplicator::wrenchCallback(geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  CachedWrench wrench;
  wrench.force = ignition::math::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
  wrench.torque = ignition::math::Vector3d(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
  wrench.receivedAt = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mWrenchMutex);
  mLatestWrenches[msg->header.frame_id] = wrench;
}

std::unordered_map<std::string, WrenchApplicator::CachedWrench> WrenchApplicator::getLatestWrenches()
{
  std::lock_guard<std::mutex> lock(mWrenchMutex);
  return mLatestWrenches;
}

WrenchApplicator::~WrenchApplicator()
{
  if (mRosSpinThread.joinable())
  {
    mRosSpinThread.join();
  }

  mRosNode = nullptr;

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  while (rclcpp::ok())
  {
    std::cout << "shutting down ROS2 plugin" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}
