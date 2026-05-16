
#include "JointPositionController.h"
#include <ignition/common/Console.hh>
#include <thread> 
#include <chrono> 
#include <vector> 


void JointPositionController::Configure(const ignition::gazebo::Entity &entity,
                                         const std::shared_ptr<const sdf::Element> &anSdf,
                                         ignition::gazebo::EntityComponentManager &ecm,
                                         ignition::gazebo::EventManager &)
{
  setRunning(true); 

  mModel = ignition::gazebo::Model(entity);
  if (!mModel.Valid(ecm))
  {
    std::cerr << "invalid model entity." << std::endl;
    return;
  }

  for(int i = 0; i < mModel.JointCount(ecm); i++)
  {
    auto jnt = std::make_shared<ignition::gazebo::Joint>(mModel.Joints(ecm)[i]); 

    if(sdf::JointType::FIXED == jnt->Type(ecm).value())
    {
      continue; 
    }

    ignmsg << "Found a " << jointTypeToString(jnt->Type(ecm).value()) << " joint with name: " << jnt->Name(ecm).value() << "\n"; 
    jnt->EnablePositionCheck(ecm, true);
    mJoints.push_back(jnt); 
  }

  if(nullptr == anSdf)
  {
    std::cerr << "missing sdf tags for JointController plugin\n"; 
    return ; 
  }

  std::string publishTopicName = "joint_positions";
  if(anSdf->HasElement("publish"))
  {
      publishTopicName = anSdf->Get<std::string>("publish");
  }

  std::string commandTopicName = "joint_commands"; 
  if(anSdf->HasElement("command"))
  {
      commandTopicName = anSdf->Get<std::string>("command"); 
  }

  ignmsg << "Publishing manipulator joint positions on /" << publishTopicName << "\n"; 
  ignmsg << "Subscribing to manipulator joint commands on /" << commandTopicName << "\n";

  auto ctx = rclcpp::contexts::get_global_default_context(); 
  
  if(!ctx->is_valid())
  {
    rclcpp::init(0, nullptr); 
  }

  mRosNode = rclcpp::Node::make_shared("joint_controller");
  mCmdSub = mRosNode->create_subscription<std_msgs::msg::Float64MultiArray>(commandTopicName, 10, std::bind(&JointPositionController::commandCallback, this, std::placeholders::_1)); 
  mPosPub = mRosNode->create_publisher<std_msgs::msg::Float64MultiArray>(publishTopicName, 10); 

  mRosSpinThread = std::thread([this](){
    rclcpp::spin(mRosNode); 
  }); 

  if(anSdf->HasElement("initial_position"))
  {
      std::string positions = anSdf->Get<std::string>("initial_position"); 
      ignmsg << "Initial positions: " << positions << std::endl;  

      mJointCommands = parseVector<double>(positions); 
  }

  // initial joint pos here  
  mPrevPosErr = mJointCommands; 
  mKp = {1000, 5000, 1000, 5000, 500, 1000, 10000, 10000}; 
  mKd = {100, 100, 100, 100, 50, 100, 0, 0}; 
  mPrevTime = std::chrono::steady_clock::now(); 

  mPublishRate = std::make_unique<RateController>(10); 
  
  mPublishThread = std::thread([&](){
    jointPositionPublishLoop(ecm); 
  });

}

void JointPositionController::PreUpdate(const ignition::gazebo::UpdateInfo&, ignition::gazebo::EntityComponentManager &ecm)
{
  auto now = std::chrono::steady_clock::now();

  std::vector<double> jntCmdCopy = getLatestJointCommand(); 
  std::vector<double> jntPosCopy;
  if(!getJointPositions(ecm, jntPosCopy))
  {
	  return; 
  }

  for(int i = 0; i < mJoints.size(); i++)
  {
      auto dt = std::chrono::duration<double>(now - mPrevTime).count(); 

      double posErr = jntCmdCopy[i] - jntPosCopy[i]; 
      double errDeriv = (posErr - mPrevPosErr[i]) / dt; 

      double forceVal = mKp[i] * posErr + mKd[i] * errDeriv; 

      std::vector<double> force(1, forceVal); 
      mJoints[i]->SetForce(ecm, force); 
      mPrevPosErr[i] = posErr;
  }
  mPrevTime = now;
}

void JointPositionController::jointPositionPublishLoop(ignition::gazebo::EntityComponentManager &ecm)
{
  while(isRunning())
  {
    mPublishRate->start(); 

    std::vector<double> jntPos; 
    if(!getJointPositions(ecm, jntPos))
    {
      continue;
    } 

    std_msgs::msg::Float64MultiArray msg; 
    msg.set__data(jntPos); 

    mPosPub->publish(msg); 
    mPublishRate->block(); 
  }

}

void JointPositionController::commandCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mCommandMutex);  
  mJointCommands = msg->data; 
}

std::vector<double> JointPositionController::getLatestJointCommand()
{
  std::lock_guard<std::mutex> lock(mCommandMutex); 
  return mJointCommands; 
}

bool JointPositionController::getJointPositions(ignition::gazebo::EntityComponentManager &ecm, std::vector<double>& aJointVecOut)
{
  for(auto jnt : mJoints)
  {
    auto optPosVec = jnt->Position(ecm); 

    if(optPosVec.has_value())
    {
      auto posVec = optPosVec.value(); 

      if(0 == posVec.size())
      {
         return false;  
      }

      aJointVecOut.push_back(posVec[0]); 

    }
  }

  return true;  
}

std::string JointPositionController::jointTypeToString(const sdf::JointType& aType)
{
  switch (aType)
  {
  case sdf::JointType::REVOLUTE:
    return "Revolute";
  default:
    return "I'm to lazy to convert the other types right now"; 
  }
}

JointPositionController::~JointPositionController()
{
  ignmsg << "###################################################################################" << std::endl; 
  ignmsg << "##################### JOINT POSITION CONTROLLER DESTRUCTOR CALLED #################" << std::endl; 
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

  ignmsg << "###################################################################################" << std::endl; 
  ignmsg << "##################### RCLCPP SHUTDOWN SUCCESSFUL ##################################" << std::endl;
  ignmsg << "###################################################################################" << std::endl;  
}
