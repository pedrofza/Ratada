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

#ifndef IMU_HPP
#define IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

class Imu
{
public:
    Imu(rclcpp::Node& parent_node, std::string topic_name);
    double MagZ() const;

private:
    rclcpp::Node& _parent_node;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
    // quaternion
    tf2::Quaternion q;
    double mag_x{0.0};
    double mag_y{0.0};
    double mag_z{0.0};
    double mag_w{0.0};

    double gyro_x{0.0};
    double gyro_y{0.0};
    double gyro_z{0.0};

    double accel_x{0.0};
    double accel_y{0.0};
    double accel_z{0.0};
    
};

#endif
