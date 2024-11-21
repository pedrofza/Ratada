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

#ifndef CMDVEL_HPP
#define CMDVEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CmdVel
{
public:
    CmdVel(rclcpp::Node& parent_node, std::string topic_name);
    void SetVel(double linear, double angular);

private:
    rclcpp::Node& _parent_node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
};

#endif
