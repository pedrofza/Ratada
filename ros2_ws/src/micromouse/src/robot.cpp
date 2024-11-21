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

#include "micromouse/robot.hpp"
#include <cmath>

SimpleRobotPoseManager::SimpleRobotPoseManager(const RobotConstructionSpecification& specs)
: m_specs(specs), m_pose{0, 0, 0}, m_calc{m_specs}
{
}

void SimpleRobotPoseManager::updatePose(int leftWheelPulses, int rightWheelPulses)
{
  Displacement d = m_calc.getDisplacement(leftWheelPulses, rightWheelPulses);

  m_pose.angle += d.angular;
  m_pose.position.x += d.linear * cos(m_pose.angle);
  m_pose.position.y += d.linear * sin(m_pose.angle);
}

//void SimpleRobotPoseManager::updatePose(int leftWheelPulses, int rightWheelPulses, unsigned long deltaTime)
//{
//  Velocity v = m_calc.getVelocity(leftWheelPulses, rightWheelPulses, deltaTime);
//  m_pose.theta += v.angular * deltaTime;
//  m_pose.x += v.linear * cos(m_pose.theta) * deltaTime;
//  m_pose.y += v.linear * sin(m_pose.theta) * deltaTime;
//}

Pose2 SimpleRobotPoseManager::getPose() const
{
  return m_pose;
}

void SimpleRobotPoseManager::setPose(Pose2 pose)
{
  m_pose = pose;
}

float SimpleRobotPoseManager::deltaDistance() const
{
  return m_pose.position.magnitude();
}

SimpleVelocityFromPulsesCalculator::SimpleVelocityFromPulsesCalculator(const RobotConstructionSpecification& specs)
: _specs(specs)
{
}

Displacement SimpleVelocityFromPulsesCalculator::getDisplacement(int leftWheelPulses, int rightWheelPulses)
{
  float leftWheelAngularDisplacement = leftWheelPulses * 2 * M_PI / _specs.leftWheelPPR;
  float rightWheelAngularDisplacement = rightWheelPulses * 2 * M_PI / _specs.rightWheelPPR;

  float leftWheelLinearDisplacement = leftWheelAngularDisplacement * _specs.leftWheelRadius;
  float rightWheelLinearDisplacement = rightWheelAngularDisplacement * _specs.rightWheelRadius;

  float robotLinearDisplacement = (rightWheelLinearDisplacement + leftWheelLinearDisplacement) / 2;
  float robotAngularDisplacement = (rightWheelLinearDisplacement - leftWheelLinearDisplacement) / _specs.wheelBaseDistance;
  
  Displacement displacement;
  displacement.linear = robotLinearDisplacement;
  displacement.angular = robotAngularDisplacement;
  return displacement;
}
