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
    ROS_INFO_STREAM("LightBar Driver Started!");
}

void LightBarApplication::initialize() 
{
    // Initialize
    nh_.reset(new ros::NodeHandle("lightbar"));

    // Configure HTTP parameters of the controller
    std::string host_name;
    nh_->param<std::string>("host_name", host_name, HTTP_HOSTNAME);
    int port;
    nh_->param<int>("port", port, HTTP_PORT);
    std::string user;
    nh_->param<std::string>("user", user, HTTP_USER);
    std::string password;
    nh_->param<std::string>("password", password, HTTP_PASSWORD);
    int status_update_rate;
    nh_->param<int>("status_update_rate", status_update_rate, 2);

    lightbar_ctrl_.configureHTTP(host_name,port,user,password);
    
    // LightbarController starts at OFF state, but make sure hardware is too.
    // Acts as a dummy request.
    try
    {
        lightbar_ctrl_.turnOffAll();
    }
    catch (CURL_EASY_PERFORM_ERROR& e)
    {
        ROS_WARN_STREAM(e.what());
        ROS_WARN_STREAM("When starting lightbar driver, could not connect to the lightbar IP.");
        cav_msgs::DriverStatus status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        return;
    }

    // Setup the ROS API
    api_.clear();
    
    // Topics 
    lightbar_pub_ = nh_->advertise<cav_msgs::LightBarStatus>("light_bar_status", 10);
    api_.push_back(lightbar_pub_.getTopic());

    // Services
    get_lights_srv_ = nh_->advertiseService("get_lights", &LightBarApplication::getLightsCB, this);
    api_.push_back(get_lights_srv_.getService());

    set_lights_srv_ = nh_->advertiseService("set_lights", &LightBarApplication::setLightsCB, this);
    api_.push_back(set_lights_srv_.getService());

    // Timer loop
    status_publisher_timer_ = nh_->createWallTimer(ros::WallDuration(ros::Rate(status_update_rate)),&LightBarApplication::updateStatusTimerCB, this);
    status_publisher_timer_.start();

    // Driver Status set to Operational
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);
    
}

bool LightBarApplication::getLightsCB(cav_srvs::GetLightsRequest&, cav_srvs::GetLightsResponse &resp) {
    static auto ON = static_cast<cav_msgs::LightBarStatus::_flash_type>(cav_msgs::LightBarStatus::ON);
    static auto OFF = static_cast<cav_msgs::LightBarStatus::_flash_type>(cav_msgs::LightBarStatus::OFF);
    
    // TODO (Optional) Make LightBarStatus service to accept LightBarID (front or back)
    // to give back status. With current design, we keep both lightbars with same state.
    LightBar curr_front;
    try
    {
        curr_front = lightbar_ctrl_.getState(FRONT_ID);
    }
    catch(CURL_EASY_PERFORM_ERROR& e)
    {
        ROS_WARN_STREAM(e.what());
        ROS_WARN_STREAM("The lightbar driver could not connect to lightbar to get light states.");
        cav_msgs::DriverStatus status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }
    catch(PARSE_ERROR& e)
    {
        ROS_WARN_STREAM(e.what());
        ROS_WARN_STREAM("Couldn't parse lightbar response from the IP.");
        cav_msgs::DriverStatus status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }
    
    //LightBar curr_back = lightbar_ctrl_.getState(BACK_ID);

    resp.status.yellow_solid    = curr_front.light_by_id[kYellowDimOn]   ? ON : OFF;
    resp.status.left_arrow  = curr_front.light_by_id[kLeftArrowOn]       ? ON : OFF;
    resp.status.right_arrow = curr_front.light_by_id[kRightArrowOn]      ? ON : OFF;
    resp.status.green_solid = curr_front.light_by_id[kGreenSolidOn]      ? ON : OFF;
    resp.status.flash = curr_front.light_by_id[kYellowFlashOn]           ? ON : OFF;
    resp.status.sides_solid = curr_front.light_by_id[kYellowSidesOn]     ? ON : OFF;

    return true;
}

bool LightBarApplication::setLightsCB(cav_srvs::SetLightsRequest &req, cav_srvs::SetLightsResponse&) 
{
    using cav_msgs::LightBarStatus;
    LightBar curr_front;

    // TODO (Optional) Make LightBarStatus service to accept LightBarID (front or back)
    // to give back status. With current design, we keep both lightbars with same state.

    try
    {
        curr_front = lightbar_ctrl_.getState(FRONT_ID);
        //LightBar curr_back = lightbar_ctrl_.getState(BACK_ID);
    }
    catch(CURL_EASY_PERFORM_ERROR& e)
    {
        ROS_WARN_STREAM(e.what());
        ROS_WARN_STREAM("The lightbar driver could not connect to lightbar to get light states.");
        cav_msgs::DriverStatus status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }
    catch(PARSE_ERROR& e)
    {
        ROS_WARN_STREAM(e.what());
        ROS_WARN_STREAM("Couldn't parse lightbar response from the IP.");
        cav_msgs::DriverStatus status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }

    curr_front.light_by_id[kYellowDimOn]  = req.set_state.yellow_solid   == LightBarStatus::ON;
    curr_front.light_by_id[kLeftArrowOn]  = req.set_state.left_arrow     == LightBarStatus::ON;
    curr_front.light_by_id[kRightArrowOn] = req.set_state.right_arrow    == LightBarStatus::ON;
    curr_front.light_by_id[kGreenSolidOn] = req.set_state.green_solid    == LightBarStatus::ON;
    curr_front.light_by_id[kYellowFlashOn]= req.set_state.flash          == LightBarStatus::ON;
    curr_front.light_by_id[kYellowSidesOn]= req.set_state.sides_solid    == LightBarStatus::ON;

    try
    {
        lightbar_ctrl_.setState(FRONT_ID, curr_front);
        lightbar_ctrl_.setState(BACK_ID, curr_front);
        //lightbar_ctrl_.setState(BACK_ID, curr_back);
    }
    catch(CURL_EASY_PERFORM_ERROR& e)
    {
        ROS_WARN_STREAM(e.what());
        ROS_WARN_STREAM("The lightbar driver could not connect to lightbar while trying to set states.");
        cav_msgs::DriverStatus status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }

    return true;
}
void LightBarApplication::updateStatusTimerCB(const ros::WallTimerEvent &) 
{
    cav_msgs::LightBarStatus light_bar_msg;
    LightBar curr_front;
    // With current design, basing the lightbar status only off of front lightbar.
    try
    {
        curr_front = lightbar_ctrl_.getState(FRONT_ID);
        //LightBar curr_back = lightbar_ctrl_.getState(BACK_ID);
    }
    catch(CURL_EASY_PERFORM_ERROR& e)
    {
        ROS_WARN_STREAM(e.what());
        ROS_WARN_STREAM("The lightbar driver could not connect to lightbar to get light states.");
        cav_msgs::DriverStatus status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        return;
    }
    catch(PARSE_ERROR& e)
    {
        ROS_WARN_STREAM(e.what());
        ROS_WARN_STREAM("Couldn't parse lightbar response from the IP.");
        cav_msgs::DriverStatus status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        return;
    }
    light_bar_msg.yellow_solid  = curr_front.light_by_id[kYellowDimOn];         
    light_bar_msg.left_arrow    = curr_front.light_by_id[kLeftArrowOn];         
    light_bar_msg.right_arrow   = curr_front.light_by_id[kRightArrowOn];        
    light_bar_msg.green_solid   = curr_front.light_by_id[kGreenSolidOn];        
    light_bar_msg.flash         = curr_front.light_by_id[kYellowFlashOn];         
    light_bar_msg.sides_solid   = curr_front.light_by_id[kYellowSidesOn];   

    lightbar_pub_.publish(light_bar_msg); 
}

void LightBarApplication::post_spin() 
{
    // bypassing any diagnostic and error handling stuff
    auto status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);
}

} // namespace lightbar_driver
