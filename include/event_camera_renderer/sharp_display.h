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

#ifndef EVENT_CAMERA_RENDERER__SHARP_DISPLAY_H_
#define EVENT_CAMERA_RENDERER__SHARP_DISPLAY_H_

#include <event_camera_codecs/decoder_factory.h>

#include "event_camera_renderer/display.h"
#include "event_camera_renderer/sharp_updater.h"

namespace event_camera_renderer
{
class SharpDisplay : public Display
{
public:
  // -------- inherited methods
  void initialize(const std::string & encoding, uint32_t width, uint32_t height) override;
  void update(const uint8_t * events, size_t numEvents) override;
  bool hasImage() const override { return (imageUpdater_.hasImage()); }
  void resetImagePtr() override { imageUpdater_.resetImagePtr(); }
  ImgPtr getImage() override { return (imageUpdater_.getImage()); }
  void setImage(ImgPtr * img) override { imageUpdater_.setImage(img); }
  // -------- end of inherited methods
  SharpDisplay() {}

private:
  SharpUpdater imageUpdater_;
  event_camera_codecs::Decoder<EventPacket, SharpUpdater> * decoder_;
  event_camera_codecs::DecoderFactory<EventPacket, SharpUpdater> decoderFactory_;
};
}  // namespace event_camera_renderer
#endif  // EVENT_CAMERA_RENDERER__SHARP_DISPLAY_H_
