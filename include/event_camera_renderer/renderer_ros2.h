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

#ifndef EVENT_CAMERA_RENDERER__RENDERER_ROS2_H_
#define EVENT_CAMERA_RENDERER__RENDERER_ROS2_H_

#include <event_camera_msgs/msg/event_packet.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "event_camera_renderer/display.h"

namespace event_camera_renderer
{
class Renderer : public rclcpp::Node
{
public:
  using EventPacket = event_camera_msgs::msg::EventPacket;
  explicit Renderer(const rclcpp::NodeOptions & options);
  ~Renderer();

private:
  void frameTimerExpired();
  void subscriptionCheckTimerExpired();
  void eventMsg(EventPacket::ConstSharedPtr msg);
  void startNewImage();
  // ------------------------  variables ------------------------------
  std::shared_ptr<Display> display_;
  rclcpp::TimerBase::SharedPtr frameTimer_;
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;
  double sliceTime_;  // duration of one frame
  rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr eventSub_;
  image_transport::Publisher imagePub_;
  sensor_msgs::msg::Image imageMsgTemplate_;
};
}  // namespace event_camera_renderer
#endif  // EVENT_CAMERA_RENDERER__RENDERER_ROS2_H_
