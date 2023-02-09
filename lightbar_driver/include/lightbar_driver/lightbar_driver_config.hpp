#pragma once

/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <iostream>
#include <vector>

namespace lightbar_driver
{

  /**
   * \brief Stuct containing the algorithm configuration values for lightbar_driver
   */
  struct Config
  { 
    // Configure HTTP parameters of the controller
    std::string host_name = "192.168.88.28";
     int port = 80;
    std::string user = "";
    std::string password = "";
    int status_update_rate = 2;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "lightbar_driver::Config { " << std::endl
           << "host_name: " << c.host_name << std::endl
           << "port: " << c.port << std::endl
           << "user: " << c.user << std::endl
           << "password: " << c.password << std::endl
           << "status_update_rate: " << c.status_update_rate << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // lightbar_driver