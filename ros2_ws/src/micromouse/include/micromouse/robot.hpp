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

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "micromouse/robot_specs.hpp"
#include "micromouse/pose2.hpp"

struct Displacement
{
    double linear;
    double angular;
};

struct Velocity
{
    double linear;
    double angular;
};

struct MazeSpecification
{
    double cellWallSize = 16.8 / 100;
    int sizeX = 16;
    int sizeY = 16;
};

struct BoundingBoxToTurningAxis
{
    double leftToAxis;
    double rightToAxis;
    double frontToAxis;
    double backToAxis;
};

class ICalculateDisplacementFromPulses
{
public:
    virtual Displacement getDisplacement(int leftWheelPulses, int rightWheelPulses) = 0;
};

class SimpleVelocityFromPulsesCalculator : public ICalculateDisplacementFromPulses
{
public:
    SimpleVelocityFromPulsesCalculator(const RobotConstructionSpecification& specs);
    Displacement getDisplacement(int leftWheelPulses, int rightWheelPulses) override;

private:
    const RobotConstructionSpecification& _specs;
};

class SimpleRobotPoseManager
{
  private:
  const RobotConstructionSpecification& m_specs;
  Pose2 m_pose;
  SimpleVelocityFromPulsesCalculator m_calc;
  
  public:
  SimpleRobotPoseManager(const RobotConstructionSpecification& specs);
//  void updatePose(int leftWheelPulses, int rightWheelPulses, unsigned long deltaTime);
  void updatePose(int leftWheelPulses, int rightWheelPulses);
  Pose2 getPose() const;
  void setPose(Pose2 pose);
  float deltaDistance() const;
};

enum class RobotMovement
{
  forward, turnLeft, turnRight, halt
};

#endif
