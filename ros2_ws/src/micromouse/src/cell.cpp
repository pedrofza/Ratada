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

#include "micromouse/cell.hpp"

bool Cell::placeWall(Direction position)
{
	bool hadWall = hasWall(position);
	int index = wallPositionToIndex(position);
	m_walls |= (1 << index);
	return !hadWall;
}

bool Cell::removeWall(Direction position)
{
	bool hadWall = hasWall(position);
	int index = wallPositionToIndex(position);
	m_walls &= ~(1 << index);
	return hadWall;
}

bool Cell::hasWall(Direction position) const
{
	int index = wallPositionToIndex(position);
	return (1 << index) & m_walls;
}

int Cell::wallPositionToIndex(Direction position)
{
    int ret = -1;
	switch(position)
	{
		case Direction::up:
			ret = 0;
			break;
		case Direction::right:
			ret = 1;
			break;
		case Direction::down:
			ret = 2;
			break;
		case Direction::left:
			ret = 3;
			break;
	}
    return ret;
}