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

#include "micromouse/pose2.hpp"
#include "micromouse/angle_utils.hpp"

double Pose2::normalizedAngle() const
{
	return angleNormalize(angle);
}

Pose2 operator-(const Pose2& lhs, const Pose2& rhs)
{
	Pose2 result;
	result.position = lhs.position - rhs.position;
	result.angle = lhs.angle - rhs.angle;
	return result;
}
