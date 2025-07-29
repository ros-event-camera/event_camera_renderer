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

#include "event_camera_renderer/renderer_ros2.h"

#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "event_camera_renderer/check_endian.h"

namespace event_camera_renderer
{
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
  double fps;
  this->get_parameter_or("fps", fps, 25.0);
  sliceTime_ = 1.0 / fps;
  imageMsgTemplate_.height = 0;
#ifdef IMAGE_TRANSPORT_USE_QOS
  const rclcpp::QoS qos(
    rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default);
#else
  const rmw_qos_profile_t qos = rmw_qos_profile_default;
#endif
  imagePub_ = image_transport::create_publisher(this, "~/image_raw", qos);
  // Since the ROS2 image transport does not call back when subscribers come and go
  // must check by polling
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

void Renderer::eventMsg(EventPacket::ConstSharedPtr msg)
{
  if (
    imageMsgTemplate_.height != msg->height || imageMsgTemplate_.width != msg->width ||
    encoding_ != msg->encoding) {
    encoding_ = msg->encoding;
    imageMsgTemplate_.header = msg->header;
    imageMsgTemplate_.width = msg->width;
    imageMsgTemplate_.height = msg->height;
    imageMsgTemplate_.encoding = "bgr8";
    imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
    imageMsgTemplate_.step = 3 * imageMsgTemplate_.width;
    startNewImage();
    display_->initialize(msg->encoding, msg->width, msg->height);
  }
  display_->update(&(msg->events[0]), msg->events.size());
}

void Renderer::frameTimerExpired()
{
  const rclcpp::Time t = this->get_clock()->now();
  // publish frame if available and somebody listening
  if (imagePub_.getNumSubscribers() != 0 && display_->hasImage()) {
    // take memory managent from image updater
    sensor_msgs::msg::Image::UniquePtr updated_img = display_->getImage();
    updated_img->header.stamp = t;
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
