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

#include "micromouse/angle_utils.hpp"
#include <cmath>

#define TWO_M_PI 2 * M_PI

double signedAngleDifference(double a, double b)
{
  double dif = b - a;
  while (dif < -M_PI) dif += TWO_M_PI;
  while (dif > M_PI) dif -= TWO_M_PI;
  return dif;
}

double angleNormalize(double angle)
{
  while (angle < 0) angle += TWO_M_PI;
  while (angle >= TWO_M_PI) angle -= TWO_M_PI;
  return angle;
}

double rad2deg(double rad)
{
	return rad * 180 / M_PI;
}

double deg2rad(double deg)
{
	return deg * M_PI / 180;
}