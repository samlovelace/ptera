
#include "VehicleController.h"
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

void VehicleController::Configure(const ignition::gazebo::Entity &entity,
					              const std::shared_ptr<const sdf::Element> &anSdf,
					              ignition::gazebo::EntityComponentManager &ecm,
					              ignition::gazebo::EventManager &)
{
    mNavRecvd = false; 
    mCmdRecvd = false; 
    mControlLoopLaunched = false; 

    setRunning(false); 

    mModel = ignition::gazebo::Model(entity); 
    if(!mModel.Valid(ecm))
    {
        std::cerr << "Invalid model entity" << std::endl; 
        return; 
    }    

    auto ctx = rclcpp::contexts::get_global_default_context(); 

    if(!ctx->is_valid())
    {
        rclcpp::init(0, nullptr); 
    }

    mRosNode = rclcpp::Node::make_shared("vehicle_controller"); 
    mCmdSub = mRosNode->create_subscription<ptera_msgs::msg::VehicleWaypoint>("/vehicle/waypoint", 10, 
                                                                             std::bind(&VehicleController::commandCallback, 
                                                                                       this, 
                                                                                       std::placeholders::_1));
    mNavSub = mRosNode->create_subscription<ptera_msgs::msg::RobotState>("/robot/pose", 10, 
                                                                         std::bind(&VehicleController::navCallback, 
                                                                                    this, 
                                                                                    std::placeholders::_1));

    mRosSpinThread = std::thread([this](){
        rclcpp::spin(mRosNode); 
    }); 

    // initial joint pos here  
    mPrevPosErr = {0, 0, 0}; 
    mKp = {10, 10, 10};  
    mKd = {20, 20, 20};  
    mPrevTime = std::chrono::steady_clock::now();
    
    // TODO: make config 
    mControlRate = std::make_unique<RateController>(10); 
    mCmdVelPub = mNode.Advertise<ignition::msgs::Twist>("/cmd_vel"); 
}

void VehicleController::PreUpdate(const ignition::gazebo::UpdateInfo&, ignition::gazebo::EntityComponentManager &ecm)
{
    if(mNavRecvd && mCmdRecvd && !mControlLoopLaunched)
    {
        setRunning(true); 

        mControlThread = std::thread([this](){
            controlLoop(); 
        }); 

        mControlLoopLaunched = true; 
    }

}

void VehicleController::setLatestNav(const ptera_msgs::msg::RobotState::SharedPtr aNavState)
{
    std::lock_guard<std::mutex> lock(mNavMutex); 
    mNav = aNavState; 
}

void VehicleController::setLatestCmd(const ptera_msgs::msg::VehicleWaypoint::SharedPtr aWaypoint)
{
    std::lock_guard<std::mutex> lock(mCmdMutex); 
    mCmd = aWaypoint; 
} 

ptera_msgs::msg::RobotState::SharedPtr VehicleController::getLatestNav()
{
    std::lock_guard<std::mutex> lock(mNavMutex); 
    return mNav; 
} 

ptera_msgs::msg::VehicleWaypoint::SharedPtr VehicleController::getLatestCmd()
{
    std::lock_guard<std::mutex> lock(mCmdMutex); 
    return mCmd; 
}

void VehicleController::commandCallback(ptera_msgs::msg::VehicleWaypoint::SharedPtr aMsg)
{
    if(!mCmdRecvd)
    {
        std::cout << "############### GOT CMD ###################" << std::endl; 
        mCmdRecvd = true; 
    }
    
    setLatestCmd(aMsg); 
}

void VehicleController::navCallback(ptera_msgs::msg::RobotState::SharedPtr aMsg)
{
    if(!mNavRecvd)
    {
        std::cout << "$$$$$$$$$$$$$$$$$$ GOT NAV $$$$$$$$$$$$$$$$$$$" << std::endl; 
        mNavRecvd = true; 
    }

    setLatestNav(aMsg); 
}

void VehicleController::controlLoop()
{
    //--------------------------------------------------------
    // Tunable Parameters
    //--------------------------------------------------------
    const double max_linear_vel   = 1.5;   // m/s
    const double max_angular_vel  = 1.2;   // rad/s

    // Hysteresis tolerances
    const double pos_tol_enter    = 0.40;  // enter FINAL_YAW mode
    const double pos_tol_exit     = 0.60;  // leave FINAL_YAW mode
    const double yaw_tol          = 0.12;  // ~7 degrees

    // Approach controller gains
    const double k_r     = 1.8;     // distance gain
    const double k_alpha = 2.4;     // turn toward goal
    const double k_beta  = -1.0;    // final heading

    // FINAL_YAW gains
    const double k_yaw   = 1.2;     // proportional yaw gain

    //--------------------------------------------------------
    // Helpers
    //--------------------------------------------------------
    auto wrap = [](double a){
        return std::atan2(std::sin(a), std::cos(a)); // (-pi, pi]
    };

    static bool in_final_yaw = false;
    static int logCounter = 0;

    while (isRunning())
    {
        mControlRate->start();

        auto cmd = getLatestCmd();
        auto nav = getLatestNav();

        if (!cmd || !nav) {
            publishTwistCmd(0,0,0);
            mControlRate->block();
            continue;
        }

        //--------------------------------------------------------
        // Extract goal yaw
        //--------------------------------------------------------
        ignition::math::Quaterniond q(cmd->orientation.w,
                                      cmd->orientation.x,
                                      cmd->orientation.y,
                                      cmd->orientation.z);
        double yaw_goal = q.Euler().Z();

        //--------------------------------------------------------
        // Position & heading errors
        //--------------------------------------------------------
        double dx = cmd->position.x - nav->position.x;
        double dy = cmd->position.y - nav->position.y;
        double r  = std::hypot(dx, dy);

        double bearing = std::atan2(dy, dx);
        double alpha   = wrap(bearing - nav->euler.yaw);
        double beta    = wrap(yaw_goal - bearing);
        double yaw_err = wrap(yaw_goal - nav->euler.yaw);

        //--------------------------------------------------------
        // Hysteresis: Manage FINAL_YAW mode
        //--------------------------------------------------------
        if (!in_final_yaw)
        {
            // Enter yaw mode when close enough
            if (r < pos_tol_enter)
                in_final_yaw = true;
        }
        else
        {
            // Leave yaw mode only if far out
            if (r > pos_tol_exit)
                in_final_yaw = false;
        }

        //--------------------------------------------------------
        // FINAL_YAW MODE (rotate only)
        //--------------------------------------------------------
        if (in_final_yaw)
        {
            // Fully done
            if (std::abs(yaw_err) < yaw_tol)
            {
                publishTwistCmd(0, 0, 0);

                if (++logCounter % 20 == 0) {
                    std::cout << "[HOLD] r=" << r 
                              << " yaw_err=" << yaw_err 
                              << " (done)" << std::endl;
                }

                mControlRate->block();
                continue;
            }

            // Slower rotation when close to target XY
            double slowdown = std::clamp(r / pos_tol_enter, 0.3, 1.0);

            double w = k_yaw * yaw_err;
            w *= slowdown;   // reduce overshoot

            w = std::clamp(w, -max_angular_vel, max_angular_vel);

            publishTwistCmd(0.0, 0.0, w);

            std::cout << "[FINAL_YAW] r=" << r 
                      << " yaw_err=" << yaw_err 
                      << " w=" << w << std::endl;

            mControlRate->block();
            continue;
        }

        //--------------------------------------------------------
        // APPROACH MODE (drive + turn)
        //--------------------------------------------------------
        double v = k_r * r * std::cos(alpha);
        double w = k_alpha * alpha + k_beta * beta;

        // Slow down when near the goal
        double slow = std::clamp(r / 1.0, 0.3, 1.0);
        v *= slow;

        v = std::clamp(v, -max_linear_vel, max_linear_vel);
        w = std::clamp(w, -max_angular_vel, max_angular_vel);

        publishTwistCmd(v, 0.0, w);

        if (++logCounter % 5 == 0) {
            std::cout << "[APPROACH] r=" << r 
                      << " alpha=" << alpha 
                      << " beta=" << beta 
                      << " v=" << v 
                      << " w=" << w 
                      << std::endl;
        }

        mControlRate->block();
    }
}

void VehicleController::publishTwistCmd(double x, double y, double h)
{
    ignition::msgs::Twist twistMsg;
    twistMsg.mutable_linear()->set_x(x);   // Forward velocity
    twistMsg.mutable_linear()->set_y(0);
    twistMsg.mutable_linear()->set_z(0.0);
    twistMsg.mutable_angular()->set_x(0.0);
    twistMsg.mutable_angular()->set_y(0.0);
    twistMsg.mutable_angular()->set_z(h);  // Yaw rotation

    mCmdVelPub.Publish(twistMsg);
}

VehicleController::~VehicleController()
{
    setRunning(false); 
    if(mRosSpinThread.joinable())
    {
        mRosSpinThread.join();
    }
    if(mControlThread.joinable())
    {
        mControlThread.join(); 
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
