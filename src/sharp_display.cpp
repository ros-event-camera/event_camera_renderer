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

#include <event_camera_renderer/sharp_display.h>

namespace event_camera_renderer
{
void SharpDisplay::initialize(const ros_compat::EventPacket & msg)
{
  decoder_ = decoderFactory_.newInstance(msg);
  if (!decoder_) {
    std::cout << "invalid encoding: " << msg.encoding << std::endl;
    throw std::runtime_error("invalid encoding!");
  }
  // create temporary decoder to find the correspondence between
  // ros time and sensor time
  auto tmp_decoder = decoderFactory_.newInstance(msg);
  uint64_t sensor_time{0};
  if (tmp_decoder->findFirstSensorTime(msg, &sensor_time)) {
    Display::updateRosToSensorTimeOffset(
      ros_compat::Time(msg.header.stamp), static_cast<int64_t>(sensor_time));
  }
}
}  // namespace event_camera_renderer
