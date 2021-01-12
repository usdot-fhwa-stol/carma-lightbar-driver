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


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ext/stdio_filebuf.h>
#include <curl/curl.h>
#include <exception>

#include "lightbar_driver/lightbar_driver_controller.h"
#include "lightbar_driver/lightbar_driver_application.h"
#include <pugixml.hpp>

namespace lightbar_driver
{

// Explicit constructor
LightBarController::LightBarController(const std::string& host_name, int port, const std::string& user, const std::string& password)
	: host_name_(host_name), port_(port), user_(user), password_(password){}

// Callback function needed for extracting response from http requests 
size_t WriteCallback(char *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

void LightBarController::configureHTTP(const std::string& host_name, int port, const std::string& user, const std::string& password)
{
	host_name_ = host_name;
	port_ = port;
	user_ = user;
	password_ = password;
	return;
}
// Send GET (if no post_data) or POST request to ip and return the response
std::string LightBarController::sendRequest(const std::string& post_data = "", const std::string& http_request = "digitaloutput/all/value")
{
	CURL *curl;

	curl = curl_easy_init();

	if (!curl) {
        throw CURL_EASY_INIT_ERROR(std::string("Error occured in function: ") + __FUNCTION__ + std::string("()\n"));
	}
	std::ostringstream urlss;
	urlss << "http://" << host_name_ << ":" << port_ << "/" << http_request;
	std::string readBuffer = "";

	curl_easy_setopt(curl, CURLOPT_URL, urlss.str().c_str());
	
	if (post_data == "") // Get request
	{
		curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
	}
	else // Post request
	{
		//char *url_encoded = curl_easy_escape(curl, post_data.c_str(), post_data.size());
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());
	}

	// Authentication
	if (user_ != "" && password_ != "") {
		std::string userpwd = user_ + ":" + password_;
		curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
		curl_easy_setopt(curl, CURLOPT_USERPWD, userpwd.c_str());
	}

	// Perform the request
	CURLcode res = curl_easy_perform(curl);
	if (res != CURLE_OK) {
		curl_easy_cleanup(curl);
		throw CURL_EASY_PERFORM_ERROR(std::string("Error occured in function: ") + __FUNCTION__ + 
			std::string("() with: ") + curl_easy_strerror(res));
	}

	// TODO for integration test:
	// Check if the request return "not implemented"

	// Return the request's response
	curl_easy_cleanup(curl);
	return readBuffer;

} // function sendRequest

// Parse string response in xml format and update front and back lightbar lights
void LightBarController::updateStatus(const std::string& response)
{
	// use pugi to parse stuff
	pugi::xml_document doc;

	auto result = doc.load_string(response.c_str());
	if (!result)
	{
		throw PARSE_ERROR(std::string("In function ") + __FUNCTION__ + 
		std::string(": Errors occured while parsing the response: ") + result.description());
	}

	auto lights = doc.child("ADAM-6256");

	if (lights)
	{
		for (pugi::xml_node light = lights.first_child(); light; light = light.next_sibling())
		{
			int light_id = std::stoi(light.child("ID").child_value());\
			// light IDs that are not used
			if (light_id == 7 || light_id == 15)
				continue;

			// LightBar hardware http requests/responses assumes 1 as off state.
			// As we use common 0:off 1:on states in code, we need to invert when parsing.
			// For example if XML response is value=1, it means OFF, so we store as 0 in our code (bitwise XOR with 1).

			if ( light_id <=6) //front
				front_.light_by_id[light_id] = std::stoi(light.child("VALUE").child_value())^1;
			else
			{
				light_id -= 8; //back
				back_.light_by_id[light_id] = std::stoi(light.child("VALUE").child_value())^1;
			}
		}
	}
	else
	{
		throw PARSE_ERROR(std::string("While parsing response from lightbar server, digital output was not found!"));
	}
	return;

} // function updateStatus

// Get current status and update the controller through sending GET request to lightbar ip
void LightBarController::getState()
{
	std::string get_response = LightBarController::sendRequest();
	LightBarController::updateStatus(get_response);
	return;
}


// Get current status of the specified lightbarID
LightBar LightBarController::getState(const LightBarID& lbid)
{
	// Get the most recent status
	getState();
	// Return the specified light bar's status
	if (lbid == FRONT_ID)
		return front_;
	else if (lbid == BACK_ID)
		return back_;
	else
		throw LIGHTBAR_ERROR(std::string("In function ") + __FUNCTION__ + std::string(": LightBarID should be either 0:front or 1:back.\n"));
}

// Get current status from the local copy of lightbarID, it is used for unit testing
LightBar LightBarController::getStateLocal(const LightBarID& lbid)
{
	// Get local state without sending GET request to lightbar ips
	if (lbid == FRONT_ID)
		return front_;
	else if (lbid == BACK_ID)
		return back_;
	else
		throw LIGHTBAR_ERROR(std::string("In function ") + __FUNCTION__ + std::string(": LightBarID should be either 0:front or 1:back.\n"));
}

// Set front(0) or back(1) lightbar according to lb, the lightbar light
void LightBarController::setState(const LightBarID& lbid, const LightBar& lb)
{
	std::string post_data = "";
	// Create http request from lightbar class,  
	// automatically inverted the request here due to hardware configuration.
	// LightBar hardware http requests/responses assumes 1 as off state.
	// As we use common 0:off 1:on states in code, we need to invert when parsing.
	// For example if in code it is off state:0, then request will be value=1 (bitwise XOR with 1).

	if (lbid == FRONT_ID)
	{
		for (std::pair<int, int> element : lb.light_by_id) 
		{
			post_data += "DO" + std::to_string(element.first) + "=" + std::to_string(element.second ^ 1);
			post_data += "&";
		}
		for (std::pair<int, int> element : back_.light_by_id) 
		{
			post_data += "DO" + std::to_string(element.first + 8) + "=" + std::to_string(element.second ^ 1);
			post_data += "&";
		}
	}
	else if (lbid == BACK_ID)
	{
		for (std::pair<int, int> element : front_.light_by_id) 
		{
			post_data += "DO" + std::to_string(element.first) + "=" + std::to_string(element.second ^ 1);
			post_data += "&";
		}
		for (std::pair<int, int> element : lb.light_by_id) 
		{
			post_data += "DO" + std::to_string(element.first + 8) + "=" + std::to_string(element.second ^ 1);
			post_data += "&";
		}
	}
	else
		throw LIGHTBAR_ERROR(std::string("In function ") + __FUNCTION__ + std::string(": LightBarID should be either 0:front or 1:back.\n"));
	
	post_data = post_data.substr(0,post_data.size() - 1);

	// Send POST request using http
	sendRequest(post_data);

	// Update local copy of lightbar only if POST was successful
	if (lbid == FRONT_ID)
		front_ = lb;
	else
		back_ = lb;
	
	return;
}

// Set individual light of front(0) or back(1) lightbar according to the lid value 1(on) or 0 (off)
void LightBarController::setState(const LightBarID& lbid, const LightID& lid, int val)
{
	// Create modified local copy of lightbar
	LightBar new_pattern;
	if (lbid == FRONT_ID)
		new_pattern = front_;
	else if (lbid == BACK_ID)
		new_pattern = back_;
	else
		throw LIGHTBAR_ERROR(std::string("In function ") + __FUNCTION__ + std::string(": LightBarID should be either 0:front or 1:back.\n"));
	new_pattern.light_by_id[lid] = val;

	// Set new status by sending request and updating local copy
	LightBarController::setState(lbid, new_pattern);

	return;
}

void LightBarController::turnOffAll()
{
	LightBar off_state(OFF,OFF,OFF,OFF,OFF,OFF,OFF);
	LightBarController::setState(FRONT_ID, off_state);
	LightBarController::setState(BACK_ID, off_state);
	return;
}

void LightBarController::turnOnAll()
{
	LightBar on_state(ON,ON,ON, ON,ON,ON,ON);
	LightBarController::setState(FRONT_ID, on_state);
	LightBarController::setState(BACK_ID, on_state);
	return;
}
} // namespace lightbar_driver