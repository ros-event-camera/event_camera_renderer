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

#ifndef EVENT_CAMERA_RENDERER__TIME_SLICE_DISPLAY_H_
#define EVENT_CAMERA_RENDERER__TIME_SLICE_DISPLAY_H_

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>
#include <event_camera_renderer/display.h>
#include <event_camera_renderer/ros_compat.h>

namespace event_camera_renderer
{
class TimeSliceDisplay : public Display, public event_camera_codecs::EventProcessor
{
public:
  TimeSliceDisplay() = default;

  // ---------- inherited from EventProcessor
  inline void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    const uint32_t offset = ex * 3 + img_->step * ey + (polarity ? 0 : 2);
    img_->data[offset] = 255;
    Display::maybeUpdateRosToSensorTimeOffset(sensor_time);
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }

  // clang-format off
  void finished() override{};
  void rawData(const char *, size_t) override{};
  // clang-format on

  // ----------- inherited from Display
  ros_compat::ImgPtr getImage() override { return (std::move(img_)); }

  void setImage(ros_compat::ImgPtr * img) override { img_ = std::move(*img); }

  void initialize(const ros_compat::EventPacket & msg) override;

  bool update(const ros_compat::EventPacket & msg, uint64_t timeLimit, uint64_t * nextTime) override
  {
    return (decoder_->decodeUntil(msg, this, timeLimit, nextTime));
  }

private:
  std::shared_ptr<event_camera_codecs::Decoder<ros_compat::EventPacket, TimeSliceDisplay>> decoder_;
  event_camera_codecs::DecoderFactory<ros_compat::EventPacket, TimeSliceDisplay> decoderFactory_;
};
}  // namespace event_camera_renderer
#endif  // EVENT_CAMERA_RENDERER__TIME_SLICE_DISPLAY_H_
