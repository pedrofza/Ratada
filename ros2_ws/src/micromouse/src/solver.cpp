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

#include "micromouse/solver.hpp"
#include <cstring>
#include <limits>

#define X 16
#define Y 16

StateTransition stateFromTo(State from, State to)
{
  int x1 = from.position.x;
  int y1 = from.position.y;
  Direction r1 = from.orientation;
  int x2 = to.position.x;
  int y2 = to.position.y;
  Direction r2 = to.orientation;
  if (x1 != x2   &&   y1 == y2   &&   r1 == r2)
  {
    Direction r = r1;
    if ((r == Direction::left && x2 == x1 - 1) || (r == Direction::right && x2 == x1 + 1))
    {
      return StateTransition::forward;
    }
  }
  else if (x1 == x2   &&   y1 != y2   &&   r1 == r2)
  {
    Direction r = r1;
    if ((r == Direction::down && y2 == y1 - 1) || (r == Direction::up && y2 == y1 + 1))
    {
      return StateTransition::forward;
    }
  }
  else if (x1 == x2   &&   y1 == y2   &&   r1 != r2)
  {
    if (r2 == ccwDirection(r1)) return StateTransition::turnLeft;
    if (r2 == cwDirection(r1)) return StateTransition::turnRight;
  }
  
  return StateTransition::turnLeft;
};

bool operator==(const State& lhs, const State& rhs)
{
     return lhs.position == rhs.position && lhs.orientation == rhs.orientation;
}

FloodFillAdvSolver::FloodFillAdvSolver(const IMaze& maze)
: _maze(maze), _seeds{false}
{
}

void FloodFillAdvSolver::addGoal(Vector2Int position)
{
    for (int i = 0; i < 4; ++i)
    {
        _seeds[position.x][position.y][i] = true;  
    }
}

void FloodFillAdvSolver::resetGoals()
{
    std::memset(_seeds, 0, sizeof(_seeds));
}

static int indexFromDirection(Direction direction)
{
    int ret = -1;
    switch(direction)
    {
    case Direction::left:
        ret = 0;
        break;
    case Direction::down:
        ret = 1;
        break;
    case Direction::right:
        ret = 2;
        break;
    case Direction::up:
        ret = 3;
        break;
    }
    return ret;
}

static Direction direction2FromInt(int orientation)
{
  switch(orientation)
  {
    case 0:
      return Direction::left;
      break;
    case 1:
      return Direction::down;
      break;
    case 2:
      return Direction::right;
      break;
    case 3:
      return Direction::up;
      break;
  }
  return Direction::left; //not supposed to happen
}

static State stateFromVars(int x, int y, int r)
{
  State s;
  s.position.x = x;
  s.position.y = y;
  s.orientation = direction2FromInt(r);
  return s;
}

bool FloodFillAdvSolver::isSeed(State state) const
{
  int x = state.position.x;
  int y = state.position.y;
  int r = indexFromDirection(state.orientation);
  return _seeds[x][y][r];
}

int FloodFillAdvSolver::cost(State state) const
{
  int x = state.position.x;
  int y = state.position.y;
  int r = indexFromDirection(state.orientation);
  return _costs[x][y][r];
}

void FloodFillAdvSolver::calculateCosts()
{
  //resets cost array;
  int initialCost = std::numeric_limits<int>::max();
  for (int i = 0; i < X; ++i)
  {
    for(int j = 0; j < Y; ++j)
    {
      for(int k = 0; k < 4; ++k)
      {
        _costs[i][j][k] = initialCost;
      }
    }
  }
  
  //assigns seeds costs 0; enqueues their backward neighbors;
  for (int i = 0; i < X; ++i)
  {
    for (int j = 0; j < Y; ++j)
    {
      for (int k = 0; k < 4; ++k)
      {
        if (_seeds[i][j][k])
        {
          _costs[i][j][k] = 0;
          State s = stateFromVars(i, j, k);
          std::vector<State> bwdNeighbors = backwardNeighbors(s);
          for (const State& s : bwdNeighbors) {
            if (!isSeed(s)) {
                q.emplace(s);
            }
          }
        }
      }
    }
  }

  // processes all 
  while (!q.empty())
  {
    State elem = q.front();
    q.pop();
    std::vector<State> fwdNeighs = forwardNeighbors(elem);
	
  	State& first = fwdNeighs[0];
  	auto fCost = cost(first);
  	for(auto n : fwdNeighs)
  	{
  		auto nc = cost(n);
  		if (nc < fCost)
  		{
  			first = n;
  			fCost = nc;
  		}
  	}
  	int lowestForwardCost = fCost;
  	/*
  	auto lowestCostNeighbor = etl::min_element(
  		fwdNeighs.begin(), 
  		fwdNeighs.end(),
  		[this] (const State& a, const State& b) -> bool { return cost(a) < cost(b); }
  	);
  	*/
	  //int lowestForwardCost = cost(*lowestCostNeighbor);
    int candidateSelfCost = lowestForwardCost + 1;
    int currentSelfCost = cost(elem);
    if (candidateSelfCost >= currentSelfCost)
    {
      continue;
    }
    
    setCost(elem, candidateSelfCost);
    std::vector<State> bwdNeighs = backwardNeighbors(elem);
    for (const State& state : bwdNeighs) {
        q.emplace(state);
    }
  }
}

void FloodFillAdvSolver::setCost(State s, int cost)
{
  int x = s.position.x;
  int y = s.position.y;
  int r = indexFromDirection(s.orientation);
  _costs[x][y][r] = cost;
}

void FloodFillAdvSolver::calculateCostsOptimized(State seedState)
{
    (void)seedState;
  calculateCosts();
}

State FloodFillAdvSolver::bestMovement(State current) const
{
    std::vector<State> fwdNeighbors = forwardNeighbors(current);
    State& first = fwdNeighbors[0];
    auto fCost = cost(first);
    for(auto n : fwdNeighbors) {
        auto nc = cost(n);
        if (nc < fCost) {
            first = n;
            fCost = nc;
        }
    }
	State& lowestCostNeighbor = first;
	int lowestForwardCost = fCost;
    (void)lowestForwardCost; 
    return lowestCostNeighbor;
}

std::vector<State> FloodFillAdvSolver::forwardNeighbors(State s) const
{
  std::vector<State> neighbors;
  State ccwNeighbor;
  ccwNeighbor.position = s.position;
  ccwNeighbor.orientation = ccwDirection(s.orientation);

  State cwNeighbor;
  cwNeighbor.position = s.position;
  cwNeighbor.orientation = cwDirection(s.orientation);
  if (!_maze.hasWall(s.position, s.orientation))
  {
    State fwdNeighbor = s;
    switch(s.orientation)
    {
      case Direction::left:
        fwdNeighbor.position.x -= 1;
        break;
      case Direction::down:
        fwdNeighbor.position.y -= 1;
        break;
      case Direction::right:
        fwdNeighbor.position.x += 1;
        break;
      case Direction::up:
        fwdNeighbor.position.y += 1;
        break;
    }
    neighbors.push_back(fwdNeighbor);
  }
  neighbors.push_back(ccwNeighbor);
  neighbors.push_back(cwNeighbor);

  return neighbors;
}

std::vector<State> FloodFillAdvSolver::backwardNeighbors(State s) const
{
  std::vector<State> neighbors;
  State ccwNeighbor;
  ccwNeighbor.position = s.position;
  ccwNeighbor.orientation = ccwDirection(s.orientation);

  State cwNeighbor;
  cwNeighbor.position = s.position;
  cwNeighbor.orientation = cwDirection(s.orientation);

  neighbors.push_back(ccwNeighbor);
  neighbors.push_back(cwNeighbor);
  
  if (!_maze.hasWall(s.position, oppositeDirection(s.orientation)))
  {
    State bwdNeighbor = s;
    switch(s.orientation)
    {
      case Direction::left:
        bwdNeighbor.position.x += 1;
        break;
      case Direction::down:
        bwdNeighbor.position.y += 1;
        break;
      case Direction::right:
        bwdNeighbor.position.x -= 1;
        break;
      case Direction::up:
        bwdNeighbor.position.y -= 1;
        break;
    }
    neighbors.push_back(bwdNeighbor);
  }
  
  return neighbors;
}

void FloodFillAdvSolver::setGoalToHome()
{
	resetGoals();
	addGoal(_maze.homePosition());
}

void FloodFillAdvSolver::setGoalToMazeGoal()
{
	resetGoals();
	for(const Vector2Int& goal : _maze.goalPositions()) {
		addGoal(goal);
	}
}

// PATH MANAGER -----------

PathManager::PathManager(const IMaze& maze)
: _solver(maze), _maze(maze), _goingToCenter{false}, _robotState{{0, 0}, Direction::up}
{
}

StateTransition PathManager::nextMovement()
{
  if (_goingToCenter && isGoal(_robotState))
  {
    _goingToCenter = false;
    _solver.setGoalToHome();
//    m_solver.calculateCosts();
  }
  else if (!_goingToCenter && isHome(_robotState))
  {
    _goingToCenter = true;
	_solver.setGoalToMazeGoal();
//    m_solver.calculateCosts();
  }
  _solver.calculateCosts();
  State nextState = _solver.bestMovement(_robotState);
  StateTransition nextMovement = stateFromTo(_robotState, nextState);
  _robotState = nextState;
//  p(nextMovement);
  return nextMovement;
}

bool PathManager::isGoal(State s) const
{
  return _solver.isSeed(s);
}

bool PathManager::isHome(State s) const
{
  static Vector2Int home;
  home.x  =  0;
  home.y = 0;
  return s.position == home;
}

void PathManager::updatedWalls()
{  
}

State PathManager::getRobotState() const
{
  return _robotState;
}
