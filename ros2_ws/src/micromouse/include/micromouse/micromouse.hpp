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

#ifndef MICROMOUSE_MICROMOUSE_HPP
#define MICROMOUSE_MICROMOUSE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <std_msgs/msg/bool.hpp>

#include "micromouse/ros/cmdvel.hpp"
#include "micromouse/ros/encoder.hpp"
#include "micromouse/ros/imu.hpp"
#include "micromouse/ros/range_sensor.hpp"
#include "micromouse/ros/dc_motor.hpp"

class MicromouseNode
    : public rclcpp::Node
{
public:
    MicromouseNode();
    ~MicromouseNode();

private:
    void update_callback();
    void update_cmd_vel(double linear, double angular);
    CmdVel cmdVel;
    DcMotor motor_left;
    DcMotor motor_right;
    RangeSensor range_center;
    RangeSensor range_right;
    RangeSensor range_left;
    Imu imu;
    Encoder encoder_left;
    Encoder encoder_right;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr micromoue_sub;
    bool micromouseStarted{false};
    rclcpp::TimerBase::SharedPtr update_timer_;
};

#endif // MICROMOUSE_MICROMOUSE_HPP
