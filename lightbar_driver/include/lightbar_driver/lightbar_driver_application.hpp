/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#ifndef _LIGHTBAR_APPLICATION_H_
#define _LIGHTBAR_APPLICATION_H_

#include "lightbar_driver/lightbar_driver_controller.hpp"
#include "lightbar_driver/lightbar_driver_config.hpp"
#include <carma_driver_msgs/msg/driver_status.hpp>
#include <carma_driver_msgs/msg/light_bar_status.hpp>
#include <carma_driver_msgs/srv/get_lights.hpp>
#include <carma_driver_msgs/srv/set_lights.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace lightbar_driver
{
class LightBarApplication :public carma_ros2_utils::CarmaLifecycleNode
{
public:
    /**
     * \brief Constructor
     */
    explicit LightBarApplication(const rclcpp::NodeOptions &);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

    carma_driver_msgs::msg::DriverStatus getStatus();

    void setStatus(const carma_driver_msgs::msg::DriverStatus status);

private:
    // API
    std::vector<std::string> api_;
    // Driver Discovery Status 
    carma_driver_msgs::msg::DriverStatus status_;

    Config config_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_driver_msgs::msg::LightBarStatus> lightbar_pub_;
    carma_ros2_utils::PubPtr<carma_driver_msgs::msg::DriverStatus> driver_discovery_pub_;
 
     // Timers
    rclcpp::TimerBase::SharedPtr lightbar_timer_;
    rclcpp::TimerBase::SharedPtr driver_discovery_timer_;

    // Service Servers
    carma_ros2_utils::ServicePtr<carma_driver_msgs::srv::GetLights> get_lights_srv_;
    carma_ros2_utils::ServicePtr<carma_driver_msgs::srv::SetLights> set_lights_srv_;
    
    // LightBar Controller Object
    LightBarController lightbar_ctrl_;

     /**
     * @brief Returns the set state of the lights
     *
     * This may not match up with the hardware, we return the current set state that was last sent
     * by the set_lights service not what the hardware is doing. If this does not match the hardware status
     * this controller will continue to attempt to set the lights.
     * @param req
     * @param resp
     * @return true always
     */
    bool getLightsCB(const std::shared_ptr<rmw_request_id_t>,carma_driver_msgs::srv::GetLights::Request::SharedPtr req, carma_driver_msgs::srv::GetLights::Response::SharedPtr resp);


    /**
     * @brief Sets the light set state
     *
     * This will set the internal variable for the desired set state of the lights. The controller will attempt
     * to command the hardware to set the lights to match this state
     * @param req
     * @param resp
     * @return true always
     */
    bool setLightsCB(const std::shared_ptr<rmw_request_id_t>,carma_driver_msgs::srv::SetLights::Request::SharedPtr req, carma_driver_msgs::srv::SetLights::Response::SharedPtr resp);

    /**
     * @brief Timer Callback that updates the lightbar status topic
     */
    void updateStatusTimerCB();

    /**
     * @brief Timer Callback that updates the driver discovery status topic
     */
    void driverDiscoveryCB(); 

    /**
     * @brief Called by the base DriverApplication class to fetch this implementation's api
     *
     * The API is a list of fully scoped names to topics and services specified by the
     * CAV Platform architecture
     *
     * @return list of api
     */
    inline std::vector<std::string>& get_api()   { return api_; }

};

} // namespace lightbar_driver


#endif /* _LIGHTBAR_APPLICATION_H_ */