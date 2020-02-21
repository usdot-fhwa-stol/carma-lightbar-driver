/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "lightbar_driver/lightbar_driver_application.h"
#include <ros/ros.h>

namespace lightbar_driver
{

LightBarApplication::LightBarApplication(int argc, char **argv) : cav::DriverApplication(argc, argv, "lightbar")
{
    cav_msgs::DriverStatus status;
    status.status = cav_msgs::DriverStatus::OFF;
    status.lightbar = true;
    setStatus(status);
    lightbar_ctrl_.turnOffAll();
    ROS_INFO_STREAM("LightBar Driver Started!");
}

void LightBarApplication::initialize() 
{
    lightbar_api_nh_.reset(new ros::NodeHandle("lightbar"));

    // Setup the ROS API
    std::string node_name = ros::this_node::getName();
    api_.clear();
    
    // Topics 
    lightbar_pub_ = lightbar_api_nh_->advertise<cav_msgs::LightBarStatus>("light_bar_status", 10);
    api_.push_back(lightbar_pub_.getTopic());

    // Services
    get_lights_srv_ = lightbar_api_nh_->advertiseService("get_lights", &LightBarApplication::getLightsCB, this);
    api_.push_back(get_lights_srv_.getService());

    set_lights_srv_ = lightbar_api_nh_->advertiseService("set_lights", &LightBarApplication::setLightsCB, this);
    api_.push_back(set_lights_srv_.getService());

    // Timer loop
    status_publisher_timer_ = nh_->createWallTimer(ros::WallDuration(ros::Rate(2)),&LightBarApplication::updateStatusTimerCB, this);
    status_publisher_timer_.start();

    // Driver Status set to Operational
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);

    spin_rate = 50;
}

bool LightBarApplication::getLightsCB(cav_srvs::GetLightsRequest&, cav_srvs::GetLightsResponse &resp) {
    static auto ON = static_cast<cav_msgs::LightBarStatus::_flash_type>(cav_msgs::LightBarStatus::ON);
    static auto OFF = static_cast<cav_msgs::LightBarStatus::_flash_type>(cav_msgs::LightBarStatus::OFF);
    
    // TODO (Optional) Make LightBarStatus service to accept LightBarID (front or back)
    // to give back status. With current design, we keep both lightbars with same state.
    LightBar curr_front = lightbar_ctrl_.getState(frontID);
    //LightBar curr_back = lightbar_ctrl_.getState(backID);

    resp.status.yellow_solid    = curr_front.light_by_id[YellowDimOn]   ? ON : OFF;
    resp.status.left_arrow  = curr_front.light_by_id[LeftArrowOn]       ? ON : OFF;
    resp.status.right_arrow = curr_front.light_by_id[RightArrowOn]      ? ON : OFF;
    resp.status.green_solid = curr_front.light_by_id[GreenSolidOn]      ? ON : OFF;
    resp.status.flash = curr_front.light_by_id[YellowFlashOn]           ? ON : OFF;
    resp.status.sides_solid = curr_front.light_by_id[YellowSidesOn]     ? ON : OFF;

    return true;
}

bool LightBarApplication::setLightsCB(cav_srvs::SetLightsRequest &req, cav_srvs::SetLightsResponse&) 
{
    using cav_msgs::LightBarStatus;

    // TODO (Optional) Make LightBarStatus service to accept LightBarID (front or back)
    // to give back status. With current design, we keep both lightbars with same state.

    LightBar curr_front = lightbar_ctrl_.getState(frontID);
    //LightBar curr_back = lightbar_ctrl_.getState(backID);
    
    curr_front.light_by_id[YellowDimOn]  = req.set_state.yellow_solid   == LightBarStatus::ON;
    curr_front.light_by_id[LeftArrowOn]  = req.set_state.left_arrow     == LightBarStatus::ON;
    curr_front.light_by_id[RightArrowOn] = req.set_state.right_arrow    == LightBarStatus::ON;
    curr_front.light_by_id[GreenSolidOn] = req.set_state.green_solid    == LightBarStatus::ON;
    curr_front.light_by_id[YellowFlashOn]= req.set_state.flash          == LightBarStatus::ON;
    curr_front.light_by_id[YellowSidesOn]= req.set_state.sides_solid    == LightBarStatus::ON;

    lightbar_ctrl_.setState(frontID, curr_front);
    lightbar_ctrl_.setState(backID, curr_front);
    //lightbar_ctrl_.setState(backID, curr_back);

    return true;
}
void LightBarApplication::updateStatusTimerCB(const ros::WallTimerEvent &) 
{
    cav_msgs::LightBarStatus light_bar_msg;
    // With current design, basing the lightbar status only off of front lightbar.
    LightBar curr_front = lightbar_ctrl_.getState(frontID);
    //LightBar curr_back = lightbar_ctrl_.getState(backID);
    
    light_bar_msg.yellow_solid  = curr_front.light_by_id[YellowDimOn];         
    light_bar_msg.left_arrow    = curr_front.light_by_id[LeftArrowOn];         
    light_bar_msg.right_arrow   = curr_front.light_by_id[RightArrowOn];        
    light_bar_msg.green_solid   = curr_front.light_by_id[GreenSolidOn];        
    light_bar_msg.flash         = curr_front.light_by_id[YellowFlashOn];         
    light_bar_msg.sides_solid   = curr_front.light_by_id[YellowSidesOn];   

    lightbar_pub_.publish(light_bar_msg); 
}

void LightBarApplication::post_spin() 
{
    // bypassing any diagnostic and error handling stuff
    auto status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);
}

void LightBarApplication::shutdown()
{
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OFF;
    setStatus(status);
    ROS_WARN_STREAM("LightBar Driver Shut Down");
}

} // namespace lightbar_driver
