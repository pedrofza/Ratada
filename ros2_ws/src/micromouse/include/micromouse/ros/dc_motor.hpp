/*
 * Copyright 2024 Pedro Fontoura Zawadniak
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef DC_MOTOR_HPP
#define DC_MOTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class DcMotor
{
public:
    DcMotor(rclcpp::Node& parent_node, std::string topic_name);
    void SetVoltage(double voltage);

private:
    rclcpp::Node& _parent_node;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub;
};

#endif
