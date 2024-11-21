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

#include <gz/msgs/double.pb.h>
#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/common/Profiler.hh>
#include <gz/math.hh>

#include <math.h>

#include "Encoder.hh"

using namespace custom;

using resolution_t = double;
using gearbox_ratio_t = double;
using EncoderMsg = gz::msgs::Int32;

class custom::EncoderPrivate {
  public: bool initialized = false;
  public: gz::transport::Node node;
  public: gz::transport::Node::Publisher pub;
  public: resolution_t resolution = 360.0;
  public: gearbox_ratio_t gearbox_ratio = 1.0;

  public: double position = 0.0;
  public: gz::sim::Entity joint{gz::sim::kNullEntity};
  public: gz::sim::Entity parentEntity{gz::sim::kNullEntity};
};

Encoder::Encoder()
  : dataPtr(std::make_unique<EncoderPrivate>())
{
}

Encoder::~Encoder()
{
}

void Encoder::SetJoint(gz::sim::Entity joint)
{
  this->dataPtr->joint = joint;
}

void Encoder::SetPosition(double position)
{
  this->dataPtr->position = position;
}

double Encoder::Position() const
{
  return this->dataPtr->position;
}

gz::sim::Entity Encoder::Joint() const
{
  return this->dataPtr->joint;
}

bool Encoder::Load(const sdf::Sensor &_sdf)
{
  if (!gz::sensors::Sensor::Load(_sdf)) {
    return false;
  }

  if (_sdf.Type() != sdf::SensorType::CUSTOM) {
    gzerr << "Attempting to a load a custom sensor, but received a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  auto type = gz::sensors::customType(_sdf);
  if ("encoder" != type)
  {
    return false;
  }
  
  if (this->Topic().empty()) {
    auto topic = "/encoder";
    gzdbg << "Topic name not set. Using default topic name: " << topic << std::endl;
    this->SetTopic(topic);
  }

  // Advertise topic where data will be published
  this->dataPtr->pub = this->dataPtr->node.Advertise<EncoderMsg>(this->Topic());
  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Encoder data for [" << this->Name() << "] advertised on [" << this->Topic() << "]" << std::endl;

  if (!_sdf.Element()->HasElement("gz:encoder"))
  {
    gzerr << "No custom configuration for [" << this->Topic() << "]" << std::endl;
    return false;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("gz:encoder");

  sdf::Errors errors;
  // Required fields
  if (!customElem->HasElement("resolution")) {
    gzerr << "Required element [resolution] not found" << std::endl;
    return false;
  }
  auto resolutionElement = customElem->GetElement("resolution");
  this->dataPtr->resolution = resolutionElement->Get<resolution_t>(errors);

  // Optional fields
  if (customElem->HasElement("gearbox_ratio")) {
    auto gearboxRatioElement = customElem->GetElement("gearbox_ratio");
    this->dataPtr->gearbox_ratio = gearboxRatioElement->Get<gearbox_ratio_t>(errors, "", dataPtr->gearbox_ratio).first;
    if (this->dataPtr->gearbox_ratio < 1.0) {
      double inv = 1.0 / this->dataPtr->gearbox_ratio;
      gzwarn << "Gearbox ratio value " << this->dataPtr->gearbox_ratio << " is below 1. Using " << inv << " instead" << std::endl;
      this->dataPtr->gearbox_ratio = inv;
    }
  }

  if (!errors.empty()) {
    gzerr << "Errors" << std::endl;
    return false;
  }

  this->dataPtr->initialized = true;
  return true;
}

bool Encoder::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("Encoder::Update");
  
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  EncoderMsg msg;
  *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

  double anglePerTick = ((2 * M_PI) / (this->dataPtr->resolution * this->dataPtr->gearbox_ratio));
  int ticks = std::round(this->Position() / anglePerTick);
  msg.set_data(ticks);

  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  return true;
}
