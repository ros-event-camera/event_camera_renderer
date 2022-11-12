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

#ifndef EVENT_ARRAY_VIEWER__DISPLAY_H_
#define EVENT_ARRAY_VIEWER__DISPLAY_H_

#include <memory>
#ifdef USING_ROS_1
#include <sensor_msgs/Image.h>
typedef std::unique_ptr<sensor_msgs::Image> ImgPtr;
#else
#include <sensor_msgs/msg/image.hpp>
using ImgPtr = sensor_msgs::msg::Image::UniquePtr;
#endif

namespace event_array_viewer
{
class Display
{
public:
  virtual ~Display() {}
  virtual void initialize(const std::string & encoding, uint32_t width, uint32_t height) = 0;
  virtual void update(const uint8_t * events, size_t numEvents) = 0;
  virtual bool hasImage() const = 0;
  virtual void resetImagePtr() = 0;
  virtual ImgPtr getImage() = 0;
  virtual void setImage(ImgPtr * img) = 0;
  // factory method
  static std::shared_ptr<Display> newInstance(const std::string & type);

protected:
  Display() {}
};
}  // namespace event_array_viewer
#endif  // EVENT_ARRAY_VIEWER__DISPLAY_H_
