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

#include "event_array_viewer/viewer_ros2.h"

#include <event_array_msgs/msg/event_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "event_array_viewer/check_endian.h"

namespace event_array_viewer
{
Viewer::Viewer(const rclcpp::NodeOptions & options)
: Node(
    "event_array_viewer",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  double fps;
  this->get_parameter_or("fps", fps, 25.0);
  sliceTime_ = 1.0 / fps;
  imageMsgTemplate_.height = 0;
  const rmw_qos_profile_t qosProf = rmw_qos_profile_default;
  imagePub_ = image_transport::create_publisher(this, "~/image_raw", qosProf);
  // Since the ROS2 image transport does not call back when subscribers come and go
  // must check by polling
  subscriptionCheckTimer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&Viewer::subscriptionCheckTimerExpired, this));
}

Viewer::~Viewer()
{
  if (frameTimer_) {
    frameTimer_->cancel();
  }
  if (subscriptionCheckTimer_) {
    subscriptionCheckTimer_->cancel();
  }
}

void Viewer::subscriptionCheckTimerExpired()
{
  // this silly dance is only necessary because ROS2 at this time does not support
  // callbacks when subscribers come and go
  if (imagePub_.getNumSubscribers()) {
    // -------------- subscribers ---------------------
    if (!imageUpdater_.hasImage()) {
      // we have subscribers but no image is being updated yet, so start doing so
      startNewImage();
    }
    if (!eventSub_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to events!");
      const int qsize = 1000;
      auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize)).best_effort().durability_volatile();
      eventSub_ = this->create_subscription<event_array_msgs::msg::EventArray>(
        "~/events", qos, std::bind(&Viewer::eventMsg, this, std::placeholders::_1));
    }
    if (!frameTimer_) {
      // start publishing frames if there is interest in either camerainfo or image
      frameTimer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(sliceTime_),
        std::bind(&Viewer::frameTimerExpired, this));
    }
  } else {
    // -------------- no subscribers -------------------
    if (eventSub_) {
      RCLCPP_INFO(this->get_logger(), "unsubscribing from events!");
      eventSub_.reset();
    }
    if (imageUpdater_.hasImage()) {
      // tell image updater to deallocate image
      imageUpdater_.resetImagePtr();
    }
    if (frameTimer_) {
      // if nobody is listening, stop publishing frames if this is currently happening
      frameTimer_->cancel();
      frameTimer_.reset();
    }
  }
}

void Viewer::eventMsg(EventArray::ConstSharedPtr msg)
{
  if (imageMsgTemplate_.height == 0) {
    imageMsgTemplate_.header = msg->header;
    imageMsgTemplate_.width = msg->width;
    imageMsgTemplate_.height = msg->height;
    imageMsgTemplate_.encoding = "bgr8";
    imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
    imageMsgTemplate_.step = 3 * imageMsgTemplate_.width;
    if (!imageUpdater_.hasImage()) {
      startNewImage();
    }
  }
  auto decoder = decoderFactory_.getInstance(msg->encoding, msg->width, msg->height);
  if (!decoder) {
    std::cout << "invalid encoding: " << msg->encoding << std::endl;
    return;
  }
  // decode will produce callbacks to imageUpdater_
  decoder->decode(&(msg->events[0]), msg->events.size(), &imageUpdater_);
}

void Viewer::frameTimerExpired()
{
  const rclcpp::Time t = this->get_clock()->now();
  // publish frame if available and somebody listening
  if (imagePub_.getNumSubscribers() != 0 && imageUpdater_.hasImage()) {
    // take memory managent from image updater
    sensor_msgs::msg::Image::UniquePtr updated_img = imageUpdater_.getImage();
    updated_img->header.stamp = t;
    // give memory management to imagePub_
    imagePub_.publish(std::move(updated_img));
    // start a new image
    startNewImage();
  }
}

void Viewer::startNewImage()
{
  if (imageMsgTemplate_.height != 0) {
    sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image(imageMsgTemplate_));
    img->data.resize(img->height * img->step, 0);  // allocate memory and set all bytes to zero
    imageUpdater_.setImage(&img);                  // event publisher will also render image now
  }
}

}  // namespace event_array_viewer

RCLCPP_COMPONENTS_REGISTER_NODE(event_array_viewer::Viewer)
