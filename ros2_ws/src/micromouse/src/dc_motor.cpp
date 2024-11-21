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

#include "micromouse/ros/dc_motor.hpp"

DcMotor::DcMotor(rclcpp::Node& parent_node, std::string topic_name)
    : _parent_node(parent_node)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    pub = _parent_node.create_publisher<std_msgs::msg::Float64>(topic_name, qos);
}

void DcMotor::SetVoltage(double voltage)
{
    std_msgs::msg::Float64 voltage_cmd;
    voltage_cmd.data = voltage;
    this->pub->publish(voltage_cmd);
}
