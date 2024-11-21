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

#ifndef POSE2_HPP
#define POSE2_HPP

#include "micromouse/vector2.hpp"

struct Pose2
{
	Vector2 position;
	double angle;
	double normalizedAngle() const;
};

Pose2 operator-(const Pose2& lhs, const Pose2& rhs);

#endif
