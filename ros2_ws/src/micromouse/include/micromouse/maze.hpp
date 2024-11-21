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

#ifndef MAZE_HPP
#define MAZE_HPP

#include "micromouse/cell.hpp"
#include "micromouse/vector2int.hpp"
#include <vector>
#include <array>

class IMaze
{
public:
    virtual int xSize() const = 0;
	virtual int ySize() const = 0;
    virtual bool hasWall(Vector2Int cellPosition, Direction direction) const = 0;
    virtual Vector2Int homePosition() const = 0;
    virtual std::vector<Vector2Int> goalPositions() const = 0;
};

class LogicMaze
    : public IMaze
{
public:
    LogicMaze(int xSize, int ySize);
    int xSize() const override;
	int ySize() const override;
	bool placeWall(Vector2Int cellPosition, Direction direction);
	bool removeWall(Vector2Int cellPosition, Direction direction);
	bool hasWall(Vector2Int cellPosition, Direction direction) const override;
    
	void setHomePosition(Vector2Int home);
	void addGoalPosition(Vector2Int goal);
    
	Vector2Int homePosition() const override;
    virtual std::vector<Vector2Int> goalPositions() const override;

private:
    int _xSize;
    int _ySize;
	Vector2Int _home;
	std::vector<Vector2Int> _goals;
    Cell *_cells;
    Vector2Int _homePosition;
    std::vector<Vector2Int> _goalPositions;
	const Cell& cellAt(Vector2Int position) const;
	      Cell& cellAt(Vector2Int position);
	bool isWithinBounds(Vector2Int position) const;
	bool hasNeighbor(Vector2Int cellPosition, Direction direction) const;
};

#endif
