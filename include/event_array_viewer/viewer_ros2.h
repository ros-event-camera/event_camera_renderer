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

#ifndef EVENT_ARRAY_VIEWER__VIEWER_ROS2_H_
#define EVENT_ARRAY_VIEWER__VIEWER_ROS2_H_

#include <event_array_codecs/decoder_factory.h>

#include <event_array_msgs/msg/event_array.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "event_array_viewer/image_updater.h"

namespace event_array_viewer
{
class Viewer : public rclcpp::Node
{
public:
  using EventArray = event_array_msgs::msg::EventArray;
  explicit Viewer(const rclcpp::NodeOptions & options);
  ~Viewer();

private:
  void frameTimerExpired();
  void subscriptionCheckTimerExpired();
  void eventMsg(EventArray::ConstSharedPtr msg);
  void startNewImage();
  // ------------------------  variables ------------------------------
  ImageUpdater imageUpdater_;
  rclcpp::TimerBase::SharedPtr frameTimer_;
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;
  double sliceTime_;  // duration of one frame
  rclcpp::Subscription<event_array_msgs::msg::EventArray>::SharedPtr eventSub_;
  image_transport::Publisher imagePub_;
  sensor_msgs::msg::Image imageMsgTemplate_;
  event_array_codecs::DecoderFactory<ImageUpdater> decoderFactory_;
};
}  // namespace event_array_viewer
#endif  // EVENT_ARRAY_VIEWER__VIEWER_ROS2_H_
