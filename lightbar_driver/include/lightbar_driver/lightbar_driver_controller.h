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


#ifndef _LIGHTBAR_CONTROLLER_H_
#define _LIGHTBAR_CONTROLLER_H_

#include <cstdint>
#include <string>
#include <netinet/in.h>
#include <unordered_map>

namespace lightbar_driver
{
// Common LightBar IDs and their meaning
enum LightBarID {FRONT_ID, BACK_ID};
enum LightID {kGreenSolidOn = 0, kGreenFlashOn = 1, kYellowDimOn = 2,kRightArrowOn = 3,kLeftArrowOn = 4,kYellowSidesOn = 5,kYellowFlashOn = 6};
enum LightState {OFF, ON};

// Custom exceptions for lightbar connection
class LIGHTBAR_ERROR : public std::exception 
{
	std::string err_;
public:
	LIGHTBAR_ERROR(const std::string& err = "LIGHTBAR_ERROR "):err_(err){};
	char const* what() const throw() {return err_.c_str();};
};
struct CURL_EASY_PERFORM_ERROR : public LIGHTBAR_ERROR { using LIGHTBAR_ERROR::LIGHTBAR_ERROR;};
struct CURL_EASY_INIT_ERROR : public LIGHTBAR_ERROR { using LIGHTBAR_ERROR::LIGHTBAR_ERROR;};
struct CURL_EASY_GETINFO_ERROR : public LIGHTBAR_ERROR { using LIGHTBAR_ERROR::LIGHTBAR_ERROR;};
struct PARSE_ERROR : public LIGHTBAR_ERROR { using LIGHTBAR_ERROR::LIGHTBAR_ERROR;};


class LightBar
{
public:
	// Turn everything on by default
	LightBar():LightBar (OFF,OFF,OFF,OFF,OFF,OFF,OFF){}              
	explicit LightBar(int green_solid_on, int green_flash_on, int yellow_dim_on, 
				int right_arrow_on, int left_arrow_on, 
				int yellow_sides_on, int yellow_flash_on)
				{
					light_by_id = {
						{kGreenSolidOn, green_solid_on},
						{kGreenFlashOn, green_flash_on},
						{kYellowDimOn, yellow_dim_on},
						{kRightArrowOn, right_arrow_on},
						{kLeftArrowOn, left_arrow_on},
						{kYellowSidesOn, yellow_sides_on},
						{kYellowFlashOn, yellow_flash_on},
					};
				}
	std::unordered_map<int, int> light_by_id;

	/**
	 * @brief LightBar equality comparison
	 * @param rhs Another LightBar object
	 * @return True iff underlying map is equal
	 * Calls an equivalence relation of map data structure 
	 * on its internal map member
	 */

	inline bool operator==(const LightBar& rhs) const
	{
		return light_by_id == rhs.light_by_id;
	}
	/// Based on operator== 
	inline bool operator!=(const LightBar& rhs) const
	{
		return !(*this == rhs);
	}
	/**
	 * @brief LightBar () assignment operator
	 * @param light_values all six values that need to be changed
	 */
	inline void operator()(int green_solid_on, int green_flash_on, int yellow_dim_on, 
				int right_arrow_on, int left_arrow_on, 
				int yellow_sides_on, int yellow_flash_on)
				{
					LightBar rhs(green_solid_on, green_flash_on, yellow_dim_on, right_arrow_on, 
						left_arrow_on, yellow_sides_on, yellow_flash_on);
					this->light_by_id = rhs.light_by_id;
				}
				
};


class LightBarController {
private:
	std::string host_name_;
	int port_;
	std::string user_;
	std::string password_;
	LightBar front_, back_;
	
public:
	LightBarController(){};
	void configureHTTP(const std::string& host_name, int port, const std::string& user, const std::string& password);
	explicit LightBarController(const std::string& host_name, int port, const std::string& user, const std::string& password);
	void getState();
	LightBar getState(const LightBarID& lbid);
	LightBar getStateLocal(const LightBarID& lbid);
	void setState(const LightBarID& lbid, const LightBar& lb);
	void setState(const LightBarID& lbid, const LightID& lid, int val);
	void turnOffAll();
	void turnOnAll();
	std::string sendRequest(const std::string& post_data, const std::string& http_request);
	void updateStatus(const std::string& response);
};

} // namespace lightbar_driver



#endif /* _LIGHTBAR_CONTROLLER_H_ */