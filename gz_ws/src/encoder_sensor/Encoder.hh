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

#ifndef ENCODER_HH_
#define ENCODER_HH_

#include <gz/sensors/Sensor.hh>
#include <gz/sim/Entity.hh>
#include <memory>

namespace custom
{
  class EncoderPrivate;

  class Encoder : public gz::sensors::Sensor
  {
  public:
    Encoder();
    virtual ~Encoder();
    virtual bool Load(const sdf::Sensor &_sdf) override;
    using Sensor::Update; // For the Update(duration, bool) definition:
    virtual bool Update(const std::chrono::steady_clock::duration &_now) override;
    void SetJoint(gz::sim::Entity joint);
    gz::sim::Entity Joint() const;
    void SetPosition(double position);
    double Position() const;

  private:
    std::unique_ptr<EncoderPrivate> dataPtr;
  };
}

#endif
