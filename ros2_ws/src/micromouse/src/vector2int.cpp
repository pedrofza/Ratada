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

#include "micromouse/vector2int.hpp"

Vector2Int Vector2Int::neighbor(Direction direction)
{
	Vector2Int result = *this;
	switch(direction)
	{
	  case Direction::up:
		result.y += 1;
		break;
	  case Direction::right:
		result.x += 1;
		break;
	  case Direction::down:
		result.y -= 1;
		break;
	  case Direction::left:
		result.x -= 1;
		break;
    }
	return result;
}

bool operator==(const Vector2Int& lhs, const Vector2Int& rhs)
{
	return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

bool operator!=(const Vector2Int& lhs, const Vector2Int& rhs)
{
	return !operator==(lhs, rhs);
}
