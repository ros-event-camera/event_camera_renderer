// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "event_camera_renderer/sharp_display.h"

namespace event_camera_renderer
{
void SharpDisplay::initialize(const std::string & encoding, uint32_t width, uint32_t height)
{
  decoder_ = decoderFactory_.getInstance(encoding, width, height);
  if (!decoder_) {
    std::cout << "invalid encoding: " << encoding << std::endl;
    throw std::runtime_error("invalid encoding!");
  }
}

void SharpDisplay::update(const uint8_t * events, size_t numEvents)
{
  // decode will produce callbacks to imageUpdater_
  decoder_->decode(events, numEvents, &imageUpdater_);
}
}  // namespace event_camera_renderer
