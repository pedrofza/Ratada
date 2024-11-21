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

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <std_msgs/msg/int32.hpp>
#include <mutex>

using ticks_t = int32_t;
using ros_msg_t = std_msgs::msg::Int32;

class Encoder
{
public:
    Encoder(rclcpp::Node& parent_node, std::string topic_name);
    ticks_t TicksDelta();
    ticks_t Ticks() const;
    void SetResolution(double resolution_);
    double Resolution() const;

private:
    rclcpp::Node& _parent_node;
    ticks_t ticks{0};
    ticks_t lastTicks{0};
    rclcpp::Subscription<ros_msg_t>::SharedPtr sub;
    double resolution;
    mutable std::mutex mutex;
};

#endif
