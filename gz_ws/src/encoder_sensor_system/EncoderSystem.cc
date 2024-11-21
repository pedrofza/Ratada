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
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/SensorFactory.hh>
#include "gz/sim/components/JointPosition.hh"
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <sdf/Sensor.hh>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "Encoder.hh"
#include "EncoderSystem.hh"

using namespace gz;
using namespace sim;

using namespace custom;

class custom::EncoderSystemPrivate {
public:
    std::unordered_map<gz::sim::Entity, std::unique_ptr<custom::Encoder>> entitySensorMap;
    gz::sensors::SensorFactory sensorFactory;
    std::unordered_set<Entity> newSensors;
    std::unordered_set<Entity> entitiesToSkip;
    bool initialized = false;
    void CreateSensors(const EntityComponentManager &_ecm);
    void Update(const EntityComponentManager &_ecm);
    void AddSensor(const EntityComponentManager &_ecm, const Entity _entity, const gz::sim::components::CustomSensor *_custom, const components::ParentEntity *_parent);
    void RemoveEncoderEntities(const gz::sim::EntityComponentManager &_ecm);
};

void EncoderSystemPrivate::CreateSensors(const EntityComponentManager &_ecm) {
    GZ_PROFILE("EncoderSystemPrivate::CreateSensors");

    if (!this->initialized) {
        _ecm.Each<gz::sim::components::CustomSensor, components::ParentEntity>(
            [&](const Entity &_entity, const gz::sim::components::CustomSensor *_custom, const components::ParentEntity *_parent) -> bool {
                this->AddSensor(_ecm, _entity, _custom, _parent);
                return true;
            }
        );
        this->initialized = true;
    } else {
        _ecm.EachNew<gz::sim::components::CustomSensor, components::ParentEntity>(
            [&](const Entity &_entity, const gz::sim::components::CustomSensor *_sensor, const components::ParentEntity *_parent) -> bool {
                this->AddSensor(_ecm, _entity, _sensor, _parent);
                return true;
            }
        );
    }
}

void EncoderSystemPrivate::Update(const EntityComponentManager &_ecm) {
    GZ_PROFILE("EncoderSystemPrivate::Update");
    _ecm.Each<gz::sim::components::CustomSensor, gz::sim::components::ParentEntity>(
        [&](const Entity &_entity, const gz::sim::components::CustomSensor * /*_custom*/, const gz::sim::components::ParentEntity *_parent) -> bool {
            if (this->entitiesToSkip.find(_entity) != this->entitiesToSkip.end()) {
                return true;
            }
            auto it = this->entitySensorMap.find(_entity);
            if (it != this->entitySensorMap.end()) {
                auto &sensor = it->second;
                auto joint = _parent->Data();
                auto jointPosition = _ecm.ComponentData<components::JointPosition>(joint);
                if (!jointPosition) {
                    gzerr << "JointPosition not found. Will be created during next PreUpdate loop execution." << std::endl;
                    return true;
                }
                auto positionVec = jointPosition.value();
                if (positionVec.size() > 1) {
                    gzerr << "More than one Dof" << std::endl;
                }
                double position = positionVec.at(0);
                sensor->SetPosition(position);
            } else {
                gzerr << "Failed to update Encoder: " << _entity << ". " << "Entity not found." << std::endl;
            }
            return true;
        }
    );
}

void EncoderSystemPrivate::AddSensor(const EntityComponentManager &_ecm, const Entity _entity, const gz::sim::components::CustomSensor *_custom, const components::ParentEntity *_parent) {
    GZ_PROFILE("EncoderSystemPrivate::AddSensor");

    // create sensor
    std::string sensorScopedName = removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
    sdf::Sensor data = _custom->Data();
    data.SetName(sensorScopedName);
    // check topic
    if (data.Topic().empty()) {
        gzdbg << "Using default topic" << std::endl;
        std::string topic = scopedName(_entity, _ecm) + "/custom";
        data.SetTopic(topic);
    }
    std::unique_ptr<custom::Encoder> sensor = this->sensorFactory.CreateSensor<custom::Encoder>(data);
    if (nullptr == sensor) {
        gzerr << "Failed to create sensor [" << sensorScopedName << "]. " << "This might be caused by the system processing a custom sensor of a different type, in which case it's safe to ignore. " << std::endl;
        gzdbg << "Addind sensor to skiplist" << std::endl;
        this->entitiesToSkip.insert(_entity);
        return;
    }
    auto joint = _parent->Data();
    if (joint == kNullEntity) {
        gzerr << "Joint not found."<< std::endl;
    }
    sensor->SetJoint(joint);

    // set sensor parent
    std::string parentName = _ecm.Component<components::Name>(_parent->Data())->Data();
    sensor->SetParent(parentName);
    this->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));
    this->newSensors.insert(_entity);
}

void EncoderSystemPrivate::RemoveEncoderEntities(const gz::sim::EntityComponentManager &_ecm) {
    GZ_PROFILE("EncoderSystemPrivate::RemoveEncoderEntities");
    _ecm.EachRemoved<gz::sim::components::CustomSensor>(
            [&](const Entity &_entity, const gz::sim::components::CustomSensor *) -> bool {
            auto sensorId = this->entitySensorMap.find(_entity);
            if (sensorId == this->entitySensorMap.end()) {
                if (this->entitiesToSkip.find(_entity) == this->entitiesToSkip.end()) {
                    gzerr << "Internal error, missing Encoder sensor for entity [" << _entity << "]" << std::endl;
                }
                return true;
            }
            this->entitySensorMap.erase(sensorId);
            return true;
        }
    );
    _ecm.EachRemoved<gz::sim::components::CustomSensor>(
            [&](const Entity &_entity, const gz::sim::components::CustomSensor *) -> bool {
            auto sensorId = this->entitiesToSkip.find(_entity);
            if (sensorId == this->entitiesToSkip.end()) {
                return true;
            }
            this->entitiesToSkip.erase(sensorId);
            return true;
        }
    );
}

EncoderSystem::EncoderSystem()
    : gz::sim::System(), dataPtr(std::make_unique<EncoderSystemPrivate>())
{
}

EncoderSystem::~EncoderSystem() = default;

void EncoderSystem::PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) {
    GZ_PROFILE("EncoderSystem::PreUpdate");

    // Create components
    for (auto entity : this->dataPtr->newSensors) {
        auto it = this->dataPtr->entitySensorMap.find(entity);
        if (it == this->dataPtr->entitySensorMap.end()) {
            gzerr << "Entity [" << entity << "] isn't in sensor map, this shouldn't happen." << std::endl;
            continue;
        }
        // Set topic
        _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));

        // Set joint
        auto jointPosition = _ecm.ComponentData<components::JointPosition>(it->second->Joint());
        if (!jointPosition && _ecm.HasEntity(it->second->Joint())) {
            _ecm.CreateComponent(it->second->Joint(), components::JointPosition());
        }
    }
    this->dataPtr->newSensors.clear();
}

void EncoderSystem::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) {
    GZ_PROFILE("EncoderSystem::PostUpdate");

    if (_info.dt < std::chrono::steady_clock::duration::zero()) {
        gzwarn << "Detected jump back in time [" << std::chrono::duration<double>(_info.dt).count() << "s]. System may not work properly." << std::endl;
    }

    this->dataPtr->CreateSensors(_ecm);

    if (!_info.paused) {
        bool needsUpdate = false;
        for (auto &it : this->dataPtr->entitySensorMap) {
            auto &sensor = it.second;
            if (sensor->NextDataUpdateTime() <= _info.simTime && sensor->HasConnections()) {
                needsUpdate = true;
                break;
            }
        }
        if (!needsUpdate) {
            return;
        }

        this->dataPtr->Update(_ecm);

        for (auto &it : this->dataPtr->entitySensorMap) {
            auto &sensor = it.second;
            // Update measurement time
            sensor->Update(_info.simTime, false);
        }
    }

    this->dataPtr->RemoveEncoderEntities(_ecm);
}

GZ_ADD_PLUGIN(
    EncoderSystem,
    gz::sim::System,
    EncoderSystem::ISystemPreUpdate,
    EncoderSystem::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(EncoderSystem, "custom::EncoderSystem")
