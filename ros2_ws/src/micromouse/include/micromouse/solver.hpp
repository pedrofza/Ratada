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

#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "micromouse/vector2int.hpp"
#include "micromouse/direction.hpp"
#include "micromouse/maze.hpp"
#include <queue>

// The states that a robot can be in.
struct State
{
    Vector2Int position;
    Direction orientation;
};

bool operator==(const State& lhs, const State& rhs);

enum class StateTransition
{
	forward, turnLeft, turnRight, invalid
};

StateTransition stateFromTo(State from, State to);

class FloodFillAdvSolver
{
public:
    FloodFillAdvSolver(const IMaze& maze);
    void addGoal(Vector2Int position);
    void setGoalToHome();
    void setGoalToMazeGoal();
    void resetGoals();
    bool isSeed(State s) const;
    int cost(State s) const;
    void calculateCosts();
    void calculateCostsOptimized(State seedState);
    State bestMovement(State current) const;

private:
    const IMaze& _maze;
    bool _seeds[16][16][4];
    int _costs[16][16][4];
    std::queue<State> q;
    void setCost(State s, int cost);
    void enqueueBackwardNeighborsIfNotSeed(State s);
    std::vector<State> backwardNeighbors(State s) const;
    std::vector<State> forwardNeighbors(State s) const;
};

class PathManager
{
public:
    PathManager(const IMaze& maze);
    StateTransition nextMovement();
    void updatedWalls();
    State getRobotState() const;

private:
    FloodFillAdvSolver _solver;
    const IMaze& _maze;
    bool _goingToCenter;
    State _robotState;
    bool isGoal(State s) const;
    bool isHome(State s) const;
};

#endif
