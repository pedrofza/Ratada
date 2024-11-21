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

#include "micromouse/ros/encoder.hpp"

Encoder::Encoder(rclcpp::Node& parent_node, std::string topic_name)
    : _parent_node(parent_node)
{
    auto cb_data = [this](ros_msg_t::UniquePtr msg) -> void {
        std::lock_guard lock(this->mutex);
        this->ticks = msg.get()->data;
    };
    this->sub = _parent_node.create_subscription<ros_msg_t>(topic_name, 10, cb_data);
    RCLCPP_INFO(_parent_node.get_logger(), "Encoder has been initialised");
}

ticks_t Encoder::Ticks() const
{
    std::lock_guard lock(this->mutex);
    return this->ticks;
}

ticks_t Encoder::TicksDelta()
{
    ticks_t ticks = Ticks();
    ticks_t delta = ticks - this->lastTicks;
    this->lastTicks = ticks;
    return delta;
}

void Encoder::SetResolution(double resolution_)
{
    this->resolution = resolution_;
}

double Encoder::Resolution() const
{
    return this->resolution;
}
