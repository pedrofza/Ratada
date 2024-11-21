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

#include "micromouse/ros/range_sensor.hpp"

RangeSensor::RangeSensor(rclcpp::Node& parent_node, std::string topic_name)
    : _parent_node(parent_node)
{
    auto cb_data = [this](sensor_msgs::msg::LaserScan::UniquePtr msg) -> void
    {
        this->lastRange = msg.get()->ranges.at(0);
    };
    this->sub = _parent_node.create_subscription<sensor_msgs::msg::LaserScan>(topic_name, 10, cb_data);
}

float RangeSensor::Read() const
{
    return this->lastRange;
}
