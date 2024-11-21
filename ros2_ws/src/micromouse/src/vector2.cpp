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

#include "micromouse/vector2.hpp"
#include <cmath>

double Vector2::magnitude() const
{
	return std::sqrt(x * x + y * y);
}

Vector2 Vector2::normalize() const
{
	return *this / magnitude();
}

double pointDistance(Vector2 a, Vector2 b)
{
	return (a - b).magnitude();
}

Vector2 operator-(const Vector2& lhs, const Vector2& rhs)
{
	Vector2 result;
	result.x = lhs.x - rhs.x;
	result.y = lhs.y - rhs.y;
	return result;
}

Vector2 operator+(const Vector2& lhs, const Vector2& rhs)
{
	Vector2 result;
	result.x = lhs.x + rhs.x;
	result.y = lhs.y + rhs.y;
	return result;
}

Vector2 operator/(const Vector2& lhs, double num)
{
	Vector2 result;
	result.x = lhs.x / num;
	result.y = lhs.y / num;
	return result;
}
