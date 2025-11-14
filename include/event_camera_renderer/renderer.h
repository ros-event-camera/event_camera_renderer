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

#ifndef EVENT_CAMERA_RENDERER__RENDERER_H_
#define EVENT_CAMERA_RENDERER__RENDERER_H_

#include <event_camera_renderer/display.h>

#include <deque>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace event_camera_renderer
{
class Renderer : public rclcpp::Node
{
public:
  using EventPacket = event_camera_msgs::msg::EventPacket;
  explicit Renderer(const rclcpp::NodeOptions & options);
  ~Renderer();

private:
  struct FrameTime
  {
    explicit FrameTime(rclcpp::Time rt, uint64_t st) : ros_time(rt), sensor_time(st) {}
    rclcpp::Time ros_time;
    uint64_t sensor_time{0};
  };
  friend std::ostream & operator<<(std::ostream & os, const Renderer::FrameTime & ft);
  class PeriodEstimator
  {
  public:
    PeriodEstimator() = default;
    double getRate() const { return (est_period_ <= 0 ? -1.0 : 1e9 / est_period_); }
    const auto & getPeriod() const { return est_period_; }
    bool update(uint64_t t);

  private:
    uint8_t num_initial_{0};
    uint64_t last_time_{0};
    double est_period_{-1.0};
  };

  void frameTimerExpired();
  void subscriptionCheckTimerExpired();
  void eventMsg(EventPacket::ConstSharedPtr msg);
  void startNewImage();
  void addNewFrame(const FrameTime & ft);
  void processEventMessages();
  void publishFrame(const FrameTime & ft);
  void resetTime();
  // ------------------------  variables ------------------------------
  std::shared_ptr<Display> display_;
  rclcpp::TimerBase::SharedPtr frameTimer_;
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;
  double sliceTime_;  // duration of one frame
  rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr eventSub_;
  image_transport::Publisher imagePub_;
  sensor_msgs::msg::Image imageMsgTemplate_;
  std::string encoding_;  // currently used incoming message encoding
  std::deque<FrameTime> frames_;
  std::queue<EventPacket::ConstSharedPtr> events_;
  PeriodEstimator framePeriod_;
  int eventQueueMemoryLimit_{0};
  size_t eventQueueMemory_{0};
};
std::ostream & operator<<(std::ostream & os, const Renderer::FrameTime & ft);
}  // namespace event_camera_renderer
#endif  // EVENT_CAMERA_RENDERER__RENDERER_H_
