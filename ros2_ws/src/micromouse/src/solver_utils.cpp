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

#include "micromouse/solver_utils.hpp"
#include <cmath>

Pose2 stateToPose(State s, double xSize, double ySize)
{
  Pose2 pose;
  pose.position.x = s.position.x * xSize;
  pose.position.y = s.position.y * ySize;
  double angle = 0.0;
  switch(s.orientation)
  {
    case Direction::left:
      angle = M_PI;
      break;
    case Direction::down:
      angle = 3 * M_PI_2;
      break;
    case Direction::right:
      angle = 0.0;
      break;
    case Direction::up:
      angle = M_PI_2;
      break;
  }
  pose.angle = angle;
  return pose;
}
