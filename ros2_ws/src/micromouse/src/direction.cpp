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

#include "micromouse/direction.hpp"

Direction oppositeDirection(Direction direction)
{
  switch(direction)
  {
	  case Direction::up:
		direction = Direction::down;
		break;
	  case Direction::right:
		direction = Direction::left;
		break;
	  case Direction::down:
		direction = Direction::up;
		break;
	  case Direction::left:
		direction = Direction::right;
		break;
  }
  return direction;
}

Direction ccwDirection(Direction direction)
{
  switch(direction)
  {
	  case Direction::up:
		direction = Direction::left;
		break;
	  case Direction::right:
		direction = Direction::up;
		break;
	  case Direction::down:
		direction = Direction::right;
		break;
	  case Direction::left:
		direction = Direction::down;
		break;
  }
  return direction;
}

Direction ccwDirection(Direction direction, int steps)
{
	if (steps < 0)
	{
		return cwDirection(direction, -steps);
	}
	steps %= 4;
	for (int i = 0; i < steps; ++i)
	{
		direction = ccwDirection(direction);
	}
	return direction;
}

Direction cwDirection(Direction direction)
{
  switch(direction)
  {
	  case Direction::up:
		direction = Direction::right;
		break;
	  case Direction::right:
		direction = Direction::down;
		break;
	  case Direction::down:
		direction = Direction::left;
		break;
	  case Direction::left:
		direction = Direction::up;
		break;
  }
  return direction;
}

Direction cwDirection(Direction direction, int steps)
{
	if(steps < 0)
	{
		return ccwDirection(direction, -steps);
	}
	steps %= 4;
	for (int i = 0; i < steps; ++i)
	{
		direction = cwDirection(direction);
	}
	return direction;
}

int ccwStepDistance(Direction from, Direction to)
{
	int stepDistance = 0;
	while (from != to)
	{
		from = ccwDirection(from);
		stepDistance++;
	}
	return stepDistance;
}

int cwStepDistance(Direction from, Direction to)
{
	int stepDistance = 0;
	while (from != to)
	{
		from = cwDirection(from);
		stepDistance++;
	}
	return stepDistance;
}

Direction localToGlobalDirection(Direction localForward, Direction subject)
{
	int localToGlobalStepOffset = ccwStepDistance(localForward, Direction::up);
	Direction result = cwDirection(subject, localToGlobalStepOffset);
	return result;
}
