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

#include "micromouse/ros/imu.hpp"
#include <tf2/LinearMath/Matrix3x3.h>

Imu::Imu(rclcpp::Node& parent_node, std::string topic_name)
    : _parent_node(parent_node)
{
    auto cb_data = [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void
    {
        this->mag_x = msg.get()->orientation.x;
        this->mag_y = msg.get()->orientation.y;
        this->mag_z = msg.get()->orientation.z;
        this->mag_w = msg.get()->orientation.w;
        this->q = tf2::Quaternion(this->mag_x, this->mag_y, this->mag_z, this->mag_w);  // Todo: ensure tf2scalar and double are compatible... How to avoid conversion between the two?

        this->gyro_x = msg.get()->angular_velocity.x;
        this->gyro_y = msg.get()->angular_velocity.y;
        this->gyro_z = msg.get()->angular_velocity.z;

        this->accel_x = msg.get()->linear_acceleration.x;
        this->accel_y = msg.get()->linear_acceleration.y;
        this->accel_z = msg.get()->linear_acceleration.z;
    };

    this->sub = _parent_node.create_subscription<sensor_msgs::msg::Imu>(topic_name, 10, cb_data);
}

double Imu::MagZ() const
{
    tf2::Matrix3x3 m{this->q};
    tf2Scalar row, pitch, yaw;
    m.getRPY(row, pitch, yaw);
    return yaw;
}
