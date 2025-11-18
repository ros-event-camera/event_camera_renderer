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

#include "event_camera_renderer/renderer.h"

#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "event_camera_renderer/check_endian.h"

namespace event_camera_renderer
{
bool Renderer::PeriodEstimator::update(uint64_t t)
{
  if (num_initial_ >= 1) {
    if (t <= last_time_) {
      return (false);
    }
    const double dt = static_cast<int64_t>(t) - static_cast<int64_t>(last_time_);
    if (num_initial_ > 1) {
      est_period_ = est_period_ * 0.9 + 0.1 * dt;
    } else {
      est_period_ = dt;
    }
  } else {
    num_initial_++;
  }
  last_time_ = t;
  return (true);
}

Renderer::Renderer(const rclcpp::NodeOptions & options)
: Node(
    "event_camera_renderer",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  std::string displayType;
  this->get_parameter_or("display_type", displayType, std::string("time_slice"));
  display_ = Display::newInstance(displayType);
  if (!display_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "invalid display type: " << displayType);
    throw std::runtime_error("invalid display type!");
  }
  this->get_parameter_or("event_queue_memory_limit", eventQueueMemoryLimit_, 10 * 1024 * 1024);

  double fps;
  this->get_parameter_or("fps", fps, 25.0);
  sliceTime_ = 1.0 / fps;
  imageMsgTemplate_.height = 0;
#ifdef IMAGE_TRANSPORT_USE_QOS
  const auto qosProf = rclcpp::SystemDefaultsQoS();
#else
  const auto qosProf = rmw_qos_profile_default;
#endif
  imagePub_ = image_transport::create_publisher(
#ifdef IMAGE_TRANSPORT_USE_NODEINTERFACE
    *this,
#else
    this,
#endif
    "~/image_raw", qosProf);

  RCLCPP_INFO(this->get_logger(), "renderer_node started up, waiting for subscribers");
  // check by polling b/c ROS2 image transport does not notify about subscription changes
  subscriptionCheckTimer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&Renderer::subscriptionCheckTimerExpired, this));
}

Renderer::~Renderer()
{
  if (frameTimer_) {
    frameTimer_->cancel();
  }
  if (subscriptionCheckTimer_) {
    subscriptionCheckTimer_->cancel();
  }
}

void Renderer::subscriptionCheckTimerExpired()
{
  // this silly dance is only necessary because ROS2 at this time does not support
  // callbacks when subscribers come and go
  if (imagePub_.getNumSubscribers()) {
    // -------------- subscribers ---------------------
    if (!display_->hasImage()) {
      // we have subscribers but no image is being updated yet, so start doing so
      startNewImage();
    }
    if (!eventSub_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to events!");
      const int qsize = 1000;
      auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize)).best_effort().durability_volatile();
      eventSub_ = this->create_subscription<event_camera_msgs::msg::EventPacket>(
        "~/events", qos, std::bind(&Renderer::eventMsg, this, std::placeholders::_1));
    }
    if (!frameTimer_) {
      // start publishing frames if there is interest in either camerainfo or image
      frameTimer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(sliceTime_),
        std::bind(&Renderer::frameTimerExpired, this));
    }
  } else {
    // -------------- no subscribers -------------------
    if (eventSub_) {
      RCLCPP_INFO(this->get_logger(), "unsubscribing from events!");
      eventSub_.reset();
    }
    if (display_->hasImage()) {
      // tell image updater to deallocate image
      display_->resetImagePtr();
    }
    if (frameTimer_) {
      // if nobody is listening, stop publishing frames if this is currently happening
      frameTimer_->cancel();
      frameTimer_.reset();
    }
  }
}

void Renderer::addNewFrame(const FrameTime & ft)
{
  if (frames_.size() >= 1000) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "frames dropped because no events are received!");
  } else {
    frames_.push_back(ft);
  }
  processEventMessages();
}

void Renderer::eventMsg(EventPacket::ConstSharedPtr msg)
{
  if (!display_->isInitialized()) {
    RCLCPP_INFO_STREAM(
      get_logger(), "initializing display for image size " << msg->width << " x " << msg->height
                                                           << " encoding: " << msg->encoding);
    encoding_ = msg->encoding;
    imageMsgTemplate_.header = msg->header;
    imageMsgTemplate_.width = msg->width;
    imageMsgTemplate_.height = msg->height;
    imageMsgTemplate_.encoding = "bgr8";
    imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
    imageMsgTemplate_.step = 3 * imageMsgTemplate_.width;
    startNewImage();
    display_->initialize(*msg);
    display_->setHeaderTime(rclcpp::Time(msg->header.stamp));
  } else {
    if (
      imageMsgTemplate_.height != msg->height || imageMsgTemplate_.width != msg->width ||
      encoding_ != msg->encoding) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "cannot change encoding type or sensor size on the fly!");
      return;
    }
  }
  if (eventQueueMemory_ + msg->events.size() < static_cast<size_t>(eventQueueMemoryLimit_)) {
    events_.push(msg);
    eventQueueMemory_ += msg->events.size();
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "event message queue full, dropping incoming event message!");
  }
  processEventMessages();
}

void Renderer::resetTime()
{
  RCLCPP_WARN(get_logger(), "time is going backwards, resetting!");
  frames_.clear();
  events_ = std::queue<EventPacket::ConstSharedPtr>();
  eventQueueMemory_ = 0;
  framePeriod_ = PeriodEstimator();
  if (display_) {
    display_->resetTime();
  }
}

void Renderer::frameTimerExpired()
{
  const rclcpp::Time t = this->get_clock()->now();
  if (!framePeriod_.update(t.nanoseconds())) {
    resetTime();  // time went backwards, reset all time related state
    return;
  }
  if (imagePub_.getNumSubscribers() == 0) {
    return;
  }
  if (display_->isInitialized()) {
    // must have ros-to-sensor time offset before adding frame
    addNewFrame(FrameTime(t, display_->rosToSensorTime(t)));
  }
}

void Renderer::processEventMessages()
{
  if (!display_->isInitialized()) {
    return;
  }
  auto & frames = frames_;
  while (!events_.empty() && !frames.empty()) {
    const auto & msg = events_.front();
    if (!display_->setHeaderTime(rclcpp::Time(msg->header.stamp))) {
      resetTime();  // will empty the event queue!
      return;
    }
    while (!frames.empty()) {
      const uint64_t time_limit = frames.front().sensor_time;
      uint64_t next_time = 0;
      if (!display_->update(*msg, time_limit, &next_time)) {
        // event message was completely decoded. Cannot emit frame yet
        // because more events may arrive that are before the frame time
        eventQueueMemory_ -= msg->events.size();
        events_.pop();
        display_->setIsFirstTimeInPacket(true);
        break;
      }
      while (!frames.empty() && frames.front().sensor_time <= next_time) {
        publishFrame(frames.front());
        frames.pop_front();
      }
    }
  }
}

void Renderer::publishFrame(const FrameTime & ft)
{
  if (imagePub_.getNumSubscribers() != 0 && display_->hasImage()) {
    // take memory managent from image updater
    sensor_msgs::msg::Image::UniquePtr updated_img = display_->getImage();
    updated_img->header.stamp = ft.ros_time;
    // give memory management to imagePub_
    imagePub_.publish(std::move(updated_img));
    // start a new image
    startNewImage();
  }
}

void Renderer::startNewImage()
{
  if (imageMsgTemplate_.height != 0) {
    sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image(imageMsgTemplate_));
    img->data.resize(img->height * img->step, 0);  // allocate memory and set all bytes to zero
    display_->setImage(&img);                      // event publisher will also render image now
  }
}

}  // namespace event_camera_renderer

RCLCPP_COMPONENTS_REGISTER_NODE(event_camera_renderer::Renderer)
