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

#ifndef _LIGHTBAR_APPLICATION_H_
#define _LIGHTBAR_APPLICATION_H_

#include "lightbar_driver/lightbar_driver_controller.h"
#include <driver_application/driver_application.h>
#include <cav_msgs/LightBarStatus.h>
#include <cav_srvs/GetLights.h>
#include <cav_srvs/SetLights.h>
#include <ros/ros.h>
#include <vector>

// LightBar Hardware Configuration Default Values
#define HTTP_HOSTNAME     "192.168.88.28"
#define HTTP_PORT         (80)
#define HTTP_USER         "root"
#define HTTP_PASSWORD     "00000000"

namespace lightbar_driver
{
class LightBarApplication: public cav::DriverApplication
{
public:
    LightBarApplication(int argc, char** argv);
    ~LightBarApplication() {}
private:

    std::vector<std::string> api_;

     //ROS
    ros::Publisher lightbar_pub_;
    ros::ServiceServer get_lights_srv_, set_lights_srv_;
    ros::WallTimer status_publisher_timer_;
    LightBarController lightbar_ctrl_;

    /**
     * @brief Initializes ROS context for this node
     *
     * Establishes the connection to the LightBar hardware. Sets up topics
     */
    virtual void initialize() override;

    /**
     * @brief Called by the base DriverApplication class after spin
     *
     * Sends messages from the outgoing queue
     */
    virtual void post_spin() override;

    /**
     * @brief Called by the base DriverApplication class prior to Spin
     *
     * Manages local state of hardware device, reconnecting as needed
     */
    virtual void pre_spin() override {}

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
    bool getLightsCB(cav_srvs::GetLightsRequest &req, cav_srvs::GetLightsResponse &resp);


    /**
     * @brief Sets the light set state
     *
     * This will set the internal variable for the desired set state of the lights. The controller will attempt
     * to command the hardware to set the lights to match this state
     * @param req
     * @param resp
     * @return true always
     */
    bool setLightsCB(cav_srvs::SetLightsRequest &req, cav_srvs::SetLightsResponse &resp);

    /**
     * @brief Timer Callback that updates the lightbar status topic
     */
    void updateStatusTimerCB(const ros::WallTimerEvent &);

    /**
     * @brief Called by the base DriverApplication class to fetch this implementation's api
     *
     * The API is a list of fully scoped names to topics and services specified by the
     * CAV Platform architecture
     *
     * @return list of api
     */
    inline virtual std::vector<std::string>& get_api() override  { return api_; }

};

} // namespace lightbar_driver


#endif /* _LIGHTBAR_APPLICATION_H_ */