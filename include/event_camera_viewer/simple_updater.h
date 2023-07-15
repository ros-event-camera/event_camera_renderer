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

#ifndef EVENT_CAMERA_VIEWER__SIMPLE_UPDATER_H_
#define EVENT_CAMERA_VIEWER__SIMPLE_UPDATER_H_

#include <memory>
#include <mutex>
#ifdef USING_ROS_1
#include <sensor_msgs/Image.h>
typedef std::unique_ptr<sensor_msgs::Image> ImgPtr;
#else
#include <sensor_msgs/msg/image.hpp>
using ImgPtr = sensor_msgs::msg::Image::UniquePtr;
#endif

#include <event_camera_codecs/event_processor.h>

namespace event_camera_viewer
{
class SimpleUpdater : public event_camera_codecs::EventProcessor
{
public:
  // ---------- inherited from EventProcessor
  inline void eventCD(uint64_t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    const uint32_t offset = ex * 3 + img_->step * ey + (polarity ? 0 : 2);
    img_->data[offset] = 255;
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}

  void finished() override{};
  void rawData(const char *, size_t) override{};
  // --------- end of inherited from EventProcessor

  // note: returns reference to pointer to allow for std::move()
  inline ImgPtr getImage() { return (std::move(img_)); }
  // check if currently building an image
  inline bool hasImage() const { return (img_ != nullptr); }

  // deallocate memory associated with pointer
  inline void resetImagePtr() { img_.reset(); }

  // take ownership of pointer and memory
  inline void setImage(ImgPtr * img) { img_ = std::move(*img); }

private:
  ImgPtr img_;
};
}  // namespace event_camera_viewer
#endif  // EVENT_CAMERA_VIEWER__SIMPLE_UPDATER_H_
