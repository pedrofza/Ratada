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

#include "micromouse/maze.hpp"
#include <cstdlib>

LogicMaze::LogicMaze(int xSize, int ySize)
    : _xSize(xSize), _ySize(ySize)
{
    this->_cells = (Cell *)std::malloc(xSize * ySize);
}

int LogicMaze::xSize() const
{
	return this->_xSize;
}

int LogicMaze::ySize() const
{
	return this->_ySize;
}

bool LogicMaze::placeWall(Vector2Int cellPosition, Direction wallPosition)
{
	if (!isWithinBounds(cellPosition)) return false;
	Cell& targetCell = cellAt(cellPosition);
	bool targetCellSuccessFlag = targetCell.placeWall(wallPosition);
	if (!hasNeighbor(cellPosition, wallPosition))
	{
		return targetCellSuccessFlag;
	}
	Vector2Int neighborPosition = cellPosition.neighbor(wallPosition);
	Cell& neighborCell = cellAt(neighborPosition);
	Direction neighborWallPosition = oppositeDirection(wallPosition);
	bool neighborCellSuccessFlag = neighborCell.placeWall(neighborWallPosition);
	return targetCellSuccessFlag && neighborCellSuccessFlag;
}

bool LogicMaze::removeWall(Vector2Int cellPosition, Direction wallPosition)
{
	if (!isWithinBounds(cellPosition)) return false;
	Cell& targetCell = cellAt(cellPosition);
	bool targetCellSuccessFlag = targetCell.removeWall(wallPosition);
	if (!hasNeighbor(cellPosition, wallPosition))
	{
		return targetCellSuccessFlag;
	}
	Vector2Int neighborPosition = cellPosition.neighbor(wallPosition);
	Cell& neighborCell = cellAt(neighborPosition);
	Direction neighborWallPosition = oppositeDirection(wallPosition);
	bool neighborCellSuccessFlag = neighborCell.removeWall(neighborWallPosition);
	return targetCellSuccessFlag && neighborCellSuccessFlag;
}

bool LogicMaze::hasWall(Vector2Int cellPosition, Direction wallPosition) const
{
	if (!isWithinBounds(cellPosition)) return false;
	const Cell& targetCell = cellAt(cellPosition);
	bool targetCellSuccessFlag = targetCell.hasWall(wallPosition);
	if (!hasNeighbor(cellPosition, wallPosition))
	{
		return targetCellSuccessFlag;
	}
	Vector2Int neighborPosition = cellPosition.neighbor(wallPosition);
	const Cell& neighborCell = cellAt(neighborPosition);
	Direction neighborWallPosition = oppositeDirection(wallPosition);
	bool neighborCellSuccessFlag = neighborCell.hasWall(neighborWallPosition);
	return targetCellSuccessFlag && neighborCellSuccessFlag;
}

Vector2Int LogicMaze::homePosition() const
{
	return _home;
}

void LogicMaze::setHomePosition(Vector2Int home)
{
	_home = home;
}

std::vector<Vector2Int> LogicMaze::goalPositions() const
{
    std::vector ret(_goals);
    return ret;
}

void LogicMaze::addGoalPosition(Vector2Int goal)
{
	_goals.push_back(goal);
}

bool LogicMaze::isWithinBounds(Vector2Int position) const
{
	return position.x >= 0 && position.x < xSize() &&
		   position.y >= 0 && position.y < ySize();
}

bool LogicMaze::hasNeighbor(Vector2Int cellPosition, Direction direction) const
{
	Vector2Int neighborPosition = cellPosition.neighbor(direction);
	return isWithinBounds(neighborPosition);
}

Cell& LogicMaze::cellAt(Vector2Int position)
{
	int x = position.x;
	int y = position.y;
    return _cells[x *ySize() + y]; // TODO: is this right?
}

const Cell& LogicMaze::cellAt(Vector2Int position) const
{
	int x = position.x;
	int y = position.y;
    return _cells[x *ySize() + y]; // TODO: is this right?
}
