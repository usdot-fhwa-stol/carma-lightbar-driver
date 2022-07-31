/*
 * Copyright (C) 2022 LEIDOS.
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

#include "lightbar_driver/lightbar_driver_application.hpp"
#include <rclcpp/rclcpp.hpp>

namespace lightbar_driver
{
  namespace std_ph = std::placeholders;

  LightBarApplication::LightBarApplication(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();
 
    // Configure HTTP parameters of the controller
    config_.host_name = declare_parameter<std::string>("host_name", config_.host_name);
    config_.port = declare_parameter<int>("port", config_.port);
    config_.user = declare_parameter<std::string>("user", user);
    config_.password = declare_parameter<std::string>("password", config_.password);
    config_.status_update_rate = declare_parameter<int>("status_update_rate", config_.status_update_rate);

    lightbar_ctrl_.configureHTTP(host_name,port,user,password);

    carma_driver_msgs::msg::DriverStatus status;
    status.status = carma_driver_msgs::msg::DriverStatus::OFF;
    status.lightbar = true;
    setStatus(status);
    RCL_INFO_STREAM(get_logger(),"LightBar Driver Started!");
  }

  carma_ros2_utils::CallbackReturn LightBarApplication::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Setup the ROS API
    api_.clear();

    // Setup Publisher 
    driver_status_pub_ = create_publisher<carma_driver_msgs::msg::DriverStatus>("driver_discovery", 1);
    api_.push_back(driver_status_pub_.getTopic());
    lightbar_pub_ = create_publisher<carma_driver_msgs::msg::LightBarStatus>("light_bar_status", 1);
    api_.push_back(lightbar_pub_.getTopic());

    // Setup service servers
    get_lights_srv_= create_service<carma_driver_msgs::srv::GetLights>("get_lights",
                                                            std::bind(&LightBarApplication::getLightsCB, this, std_ph::_1, std_ph::_2, std_ph::_3));
    api_.push_back(get_lights_srv_.getService());
  
    set_lights_srv_ = create_service<carma_driver_msgs::srv::SetLights>("set_lights",
                                                            std::bind(&LightBarApplication::setLightsCB, this, std_ph::_1, std_ph::_2, std_ph::_3));
    api_.push_back(set_lights_srv_.getService());


    // LightbarController starts at OFF state, but make sure hardware is too.
    // Acts as a dummy request.
    try
    {
        lightbar_ctrl_.turnOffAll();
    }
    catch (CURL_EASY_PERFORM_ERROR& e)
    {
        RCLCPP_WARN_STREAM(get_logger(),e.what());
        RCLCPP_WARN_STREAM(get_logger(),"When starting lightbar driver, could not connect to the lightbar IP: " + host_name);
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
        setStatus(status);
        return;
    }
   
    // Driver Status set to Operational
    carma_driver_msgs::msg::DriverStatus status = getStatus();
    status.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
    setStatus(status);

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  } 

    
carma_ros2_utils::CallbackReturn LightBarApplication::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
 {
        //Timer setup - Light Bar
        lightbar_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
        std::bind(&LightBarApplication::updateStatusTimerCB, this));

        //Timer setup - Driver Discovery
        driver_discovery_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
        std::bind(&LightBarApplication::driverDiscoveryCB, this));
        
        return CallbackReturn::SUCCESS;
 }
 
bool LightBarApplication::getLightsCB(const std::shared_ptr<rmw_request_id_t>,carma_driver_msgs::srv::GetLights::Request::SharedPtr req, carma_driver_msgs::srv::GetLights::Response::SharedPtr resp) {
    static auto ON = static_cast<carma_driver_msgs::msg::LightBarStatus::_flash_type>(carma_driver_msgs::msg::LightBarStatus::ON);
    static auto OFF = static_cast<carma_driver_msgs::msg::LightBarStatus::_flash_type>(carma_driver_msgs::msg::LightBarStatus::OFF);
    
    // TODO (Optional) Make LightBarStatus service to accept LightBarID (front or back)
    // to give back status. With current design, we keep both lightbars with same state.
    LightBar curr_front;
    try
    {
        curr_front = lightbar_ctrl_.getState(FRONT_ID);
    }
    catch(CURL_EASY_PERFORM_ERROR& e)
    {
        RCLCPP_WARN_STREAM(get_logger(),e.what());
        RCLCPP_WARN_STREAM(get_logger(),"The lightbar driver could not connect to lightbar to get light states.");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }
    catch(PARSE_ERROR& e)
    {
        RCLCPP_WARN_STREAM(get_logger(),e.what());
        RCLCPP_WARN_STREAM(get_logger(),"Couldn't parse lightbar response from the IP.");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }
    
    //LightBar curr_back = lightbar_ctrl_.getState(BACK_ID);

    resp->status.yellow_solid    = curr_front.light_by_id[kYellowDimOn]   ? ON : OFF;
    resp->status.left_arrow  = curr_front.light_by_id[kLeftArrowOn]       ? ON : OFF;
    resp->status.right_arrow = curr_front.light_by_id[kRightArrowOn]      ? ON : OFF;
    resp->status.green_solid = curr_front.light_by_id[kGreenSolidOn]      ? ON : OFF;
    resp->status.green_flash = curr_front.light_by_id[kGreenFlashOn]      ? ON : OFF;
    resp->status.flash = curr_front.light_by_id[kYellowFlashOn]           ? ON : OFF;
    resp->status.sides_solid = curr_front.light_by_id[kYellowSidesOn]     ? ON : OFF;

    return true;
}

bool LightBarApplication::setLightsCB(const std::shared_ptr<rmw_request_id_t>,carma_driver_msgs::srv::SetLights::Request::SharedPtr req, carma_driver_msgs::srv::SetLights::Response::SharedPtr resp) 
{
    using carma_driver_msgs::msg::LightBarStatus;
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
        RCLCPP_WARN_STREAM(get_logger(),e.what());
        RCLCPP_WARN_STREAM(get_logger(),"The lightbar driver could not connect to lightbar to get light states.");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }
    catch(PARSE_ERROR& e)
    {
        RCLCPP_WARN_STREAM(e.what());
        RCLCPP_WARN_STREAM(get_logger(),"Couldn't parse lightbar response from the IP.");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }

    curr_front.light_by_id[kYellowDimOn]  = req->set_state.yellow_solid   == LightBarStatus::ON;
    curr_front.light_by_id[kLeftArrowOn]  = req->set_state.left_arrow     == LightBarStatus::ON;
    curr_front.light_by_id[kRightArrowOn] = req->set_state.right_arrow    == LightBarStatus::ON;
    curr_front.light_by_id[kGreenSolidOn] = req->set_state.green_solid    == LightBarStatus::ON;
    curr_front.light_by_id[kGreenFlashOn] = req->set_state.green_flash    == LightBarStatus::ON;
    curr_front.light_by_id[kYellowFlashOn]= req->set_state.flash          == LightBarStatus::ON;
    curr_front.light_by_id[kYellowSidesOn]= req->set_state.sides_solid    == LightBarStatus::ON;

    try
    {
        lightbar_ctrl_.setState(FRONT_ID, curr_front);
        lightbar_ctrl_.setState(BACK_ID, curr_front);
        //lightbar_ctrl_.setState(BACK_ID, curr_back);
    }
    catch(CURL_EASY_PERFORM_ERROR& e)
    {
        RCLCPP_WARN_STREAM(e.what());
        RCLCPP_WARN_STREAM("The lightbar driver could not connect to lightbar while trying to set states.");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
        setStatus(status);
        return false;
    }

    return true;
}

void LightBarApplication::updateStatusTimerCB() 
{
    carma_driver_msgs::msg::LightBarStatus light_bar_msg;
    LightBar curr_front;
    // With current design, basing the lightbar status only off of front lightbar.
    try
    {
        curr_front = lightbar_ctrl_.getState(FRONT_ID);
        //LightBar curr_back = lightbar_ctrl_.getState(BACK_ID);
    }
    catch(CURL_EASY_PERFORM_ERROR& e)
    {
        RCLCPP_WARN_STREAM(get_logger(),e.what());
        RCLCPP_WARN_STREAM("The lightbar driver could not connect to lightbar to get light states.");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
        setStatus(status);
        return;
    }
    catch(PARSE_ERROR& e)
    {
        RCLCPP_WARN_STREAM(get_logger(),e.what());
        RCLCPP_WARN_STREAM(get_logger(),"Couldn't parse lightbar response from the IP.");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
        setStatus(status);
        return;
    }
    light_bar_msg.yellow_solid  = curr_front.light_by_id[kYellowDimOn];         
    light_bar_msg.left_arrow    = curr_front.light_by_id[kLeftArrowOn];         
    light_bar_msg.right_arrow   = curr_front.light_by_id[kRightArrowOn];        
    light_bar_msg.green_solid   = curr_front.light_by_id[kGreenSolidOn]; 
    light_bar_msg.green_flash   = curr_front.light_by_id[kGreenFlashOn];        
    light_bar_msg.flash         = curr_front.light_by_id[kYellowFlashOn];         
    light_bar_msg.sides_solid   = curr_front.light_by_id[kYellowSidesOn];   

    lightbar_pub_.publish(light_bar_msg);

    // bypassing any diagnostic and error handling stuff
    auto status = getStatus();
    status.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
    setStatus(status); 
}

void LightBarApplication::driverDiscoveryCB() 
{
    auto driver_status = getStatus();
    driver_status_pub_.publish(driver_status); 
}

carma_driver_msgs::msg::DriverStatus LightBarApplication::getStatus()
{
 return status_;
}

void LightBarApplication::setStatus(carma_driver_msgs::msg::DriverStatus status)
{
 status_ = status;
}

} // namespace lightbar_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(lightbar_driver::LightBarApplication)


