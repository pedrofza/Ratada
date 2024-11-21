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

#include "micromouse/maze_utils.hpp"

LogicMaze classicMicromouseMaze()
{
	LogicMaze maze(16, 16);
	surroundMazeWithWalls(maze);
	maze.placeWall({0, 0}, Direction::right);
	maze.setHomePosition({0, 0});
	maze.addGoalPosition({7, 7});
	maze.addGoalPosition({7, 8});
	maze.addGoalPosition({8, 7});
	maze.addGoalPosition({8, 8});
	return maze;
}

void surroundMazeWithWalls(LogicMaze& maze)
{
  for (auto i = 0; i < maze.xSize(); ++i)
  {
    for (auto j = 0; j < maze.ySize(); ++j)
    {
      Vector2Int cellPosition = {i, j};
      if (i == 0)
      {
        maze.placeWall(cellPosition, Direction::left);
      }
      else if (i == maze.xSize() - 1)
      {
        maze.placeWall(cellPosition, Direction::right);
      }
      if (j == 0)
      {
        maze.placeWall(cellPosition, Direction::down);
      }
      else if (j == maze.ySize() - 1)
      {
        maze.placeWall(cellPosition, Direction::up);
      }
    }
  }
}
