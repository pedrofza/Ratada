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

#ifndef RANGE_SENSOR_HPP
#define RANGE_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class RangeSensor
{
public:
    RangeSensor(rclcpp::Node& parent_node, std::string topic_name);
    float Read() const;

private:
    rclcpp::Node& _parent_node;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
    float lastRange;
};

#endif
