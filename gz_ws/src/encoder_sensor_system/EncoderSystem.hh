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

#ifndef ENCODERSYSTEM_HH_
#define ENCODERSYSTEM_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace custom {
class EncoderSystemPrivate;

class EncoderSystem : public gz::sim::System,
                      public gz::sim::ISystemPreUpdate,
                      public gz::sim::ISystemPostUpdate {
public:
    explicit EncoderSystem();
    ~EncoderSystem() override;
    void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;
    void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) final;

private:
    std::unique_ptr<EncoderSystemPrivate> dataPtr;
};
} // namespace custom

#endif
