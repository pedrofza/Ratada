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

#include "micromouse/math_utils.hpp"
#include <cmath>

Vector2 rayCast(Vector2 origin, double angle, double length)
{
	origin.x += length * std::cos(angle);
	origin.y += length * std::sin(angle);
	return origin;
}

bool isWithinRectangle(Vector2 point, Vector2 recSize)
{
	return point.x >= -recSize.x/2 && point.x <= recSize.x/2
		&& point.y >= -recSize.y/2 && point.y <= recSize.y/2;
}

bool isWithinRectangle(Vector2 point, Vector2 recCenter, Vector2 recSize)
{
	double forwardBoundary  = recCenter.y + recSize.y / 2;
	double rightBoundary    = recCenter.x + recSize.x / 2;
	double backwardBoundary = recCenter.y - recSize.y / 2;
	double leftBoundary     = recCenter.x - recSize.x / 2;
	
	return (point.x >= leftBoundary     && point.x < rightBoundary  )
		&& (point.y >= backwardBoundary && point.y < forwardBoundary);
}

bool isWithinCircle(Vector2 point, double radius)
{
	return point.magnitude() <= radius;
}
