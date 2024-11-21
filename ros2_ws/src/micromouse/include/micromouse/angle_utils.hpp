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

#ifndef ANGLE_ULTILS_H
#define ANGLE_ULTILS_H

// returns b - a, constrained to [-PI, PI]
double signedAngleDifference(double a, double b);

//returns angle constrained to [0, 2*PI[
double angleNormalize(double angle);

double rad2deg(double rad);
double deg2rad(double deg);

#endif
