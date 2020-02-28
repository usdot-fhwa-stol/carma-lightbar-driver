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


#include <gtest/gtest.h>
#include <lightbar_driver/lightbar_driver_controller.h>

namespace lightbar_driver
{

TEST(LightParamTest, test)
{
    // Initialize
    LightBar on_bar(ON,ON,ON,ON,ON,ON);
    LightBar off_bar;
    // testing ==
    ASSERT_FALSE(on_bar == off_bar);
    // testing !=
    ASSERT_TRUE(on_bar != off_bar);
    on_bar.light_by_id[kGreenSolidOn] = OFF;
    ASSERT_FALSE(on_bar == off_bar);
    on_bar.light_by_id[kGreenSolidOn] = ON;
    // off_bar is all on now
    // on_bar is all on now as well
    off_bar = on_bar;
    // testing assignment= operator
    ASSERT_TRUE(off_bar == on_bar);
    // testing operator()
    on_bar.light_by_id[kGreenSolidOn] = OFF;
    ASSERT_FALSE(off_bar == on_bar);
    off_bar(OFF,ON,ON,ON,ON,ON);
    ASSERT_TRUE(off_bar == on_bar);
}

TEST (GetStateTest, testException)
{
    LightBarController lbc;
    LightBar off_state;
    // Without hardware, should not be able to connect
    ASSERT_THROW(lbc.getState(FRONT_ID), CURL_EASY_PERFORM_ERROR);
    // There should not be any change in the local copy
    // lbc is created with both lights off by default
    ASSERT_TRUE(off_state == lbc.getStateLocal(FRONT_ID));
    ASSERT_TRUE(off_state == lbc.getStateLocal(BACK_ID));
}

TEST (SetStateTest, testException)
{
    LightBarController lbc;
    LightBar off_state, on_state(ON,ON,ON,ON,ON,ON);
    // Without hardware, should not be able to connect
    ASSERT_THROW(lbc.setState(FRONT_ID, on_state), CURL_EASY_PERFORM_ERROR);
    // There should not be any change in the local copy
    // lbc is created with both lights off by default
    ASSERT_TRUE(off_state == lbc.getStateLocal(FRONT_ID));
    ASSERT_TRUE(off_state == lbc.getStateLocal(BACK_ID));
}
/*
TODO Need mock localhost listener to implement this.
TEST (SendRequestTest, test1)
{
    LightBarController lbc;
    LightBar off_state, on_state(ON,ON,ON,ON,ON,ON);
    std::string post_data = "", http_request = "api/v1/employees";
    // Without hardware, should not be able to connect
    ASSERT_THROW(lbc.sendRequest(FRONT_ID, on_state), CURL_EASY_PERFORM_ERROR);
    // There should not be any change in the local copy
    // lbc is created with both lights off by default
    ASSERT_TRUE(off_state == lbc.getStateLocal(FRONT_ID));
    ASSERT_TRUE(off_state == lbc.getStateLocal(BACK_ID));
}
*/
TEST (UpdateStatusTest, test1)
{
    LightBarController lbc;
    // Sample xml response from GET request

    // test if updateStatus can update using XML string, and
    // test if it updates light IDs only are in the string
	std::string response_str = "<ADAM-6256 status=\"OK\"><DO><ID>0</ID><VALUE>1</VALUE></DO><DO><ID>2</ID><VALUE>0</VALUE></DO><DO><ID>3</ID><VALUE>0</VALUE></DO></ADAM-6256> ";
    LightBar response_front(OFF, ON, ON, OFF, OFF, OFF);
    LightBar response_back;
	// Update according to string XML response
	lbc.updateStatus(response_str);
    LightBar front = lbc.getStateLocal(FRONT_ID);
    LightBar back = lbc.getStateLocal(BACK_ID);
    ASSERT_TRUE (front == response_front);
    ASSERT_TRUE (back == response_back);

    // test if updateStatus can update two lightbars at the same time
    response_str = "<ADAM-6256 status=\"OK\"><DO><ID>0</ID><VALUE>1</VALUE></DO><DO><ID>1</ID><VALUE>0</VALUE></DO><DO><ID>9</ID><VALUE>0</VALUE></DO><DO><ID>8</ID><VALUE>0</VALUE></DO></ADAM-6256> ";
    response_front(OFF, ON, ON, OFF, OFF, OFF);
    response_back(ON,OFF,OFF,OFF,OFF,OFF);
    lbc.updateStatus(response_str);
    front = lbc.getStateLocal(FRONT_ID);
    back = lbc.getStateLocal(BACK_ID);
    ASSERT_TRUE (front == response_front);
    ASSERT_TRUE (back == response_back);
}

} //namespace lightbar_driver





