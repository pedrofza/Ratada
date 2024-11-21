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

#ifndef CELL_HPP
#define CELL_HPP

#include "micromouse/direction.hpp"
#include <cstdint>

class Cell
{
public:
    static int wallPositionToIndex(Direction direction);
    bool placeWall(Direction direction);
    bool removeWall(Direction direction);
    bool hasWall(Direction direction) const;

private:
    std::uint8_t m_walls;
};

#endif
