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

#include "event_array_viewer/viewer_ros1.h"

#include <event_array_codecs/decoder.h>

#include "event_array_viewer/check_endian.h"

namespace event_array_viewer
{
namespace ph = std::placeholders;
Viewer::Viewer(ros::NodeHandle & nh) : nh_(nh)
{
  sliceTime_ = 1.0 / nh_.param<double>("fps", 25.0);
  image_transport::ImageTransport it(nh_);
  imagePub_ = it.advertise(
    "image_raw", 1, boost::bind(&Viewer::imageConnectCallback, this, boost::placeholders::_1),
    boost::bind(&Viewer::imageConnectCallback, this, boost::placeholders::_1));
  imageMsgTemplate_.height = 0;
}

Viewer::~Viewer() { frameTimer_.stop(); }

void Viewer::imageConnectCallback(const image_transport::SingleSubscriberPublisher &)
{
  if (imagePub_.getNumSubscribers() != 0) {
    if (!isSubscribedToEvents_) {
      frameTimer_ = nh_.createTimer(ros::Duration(sliceTime_), &Viewer::frameTimerExpired, this);
      eventSub_ = nh_.subscribe("events", 1000 /*qsize */, &Viewer::eventMsg, this);
      isSubscribedToEvents_ = true;
    }
  } else {
    if (isSubscribedToEvents_) {
      eventSub_.shutdown();  // unsubscribe
      isSubscribedToEvents_ = false;
      frameTimer_.stop();
    }
  }
}

void Viewer::frameTimerExpired(const ros::TimerEvent &)
{
  if (imagePub_.getNumSubscribers() != 0 && imageUpdater_.hasImage()) {
    // take memory managent from image updater
    std::unique_ptr<sensor_msgs::Image> updated_img = imageUpdater_.getImage();
    updated_img->header.stamp = ros::Time::now();
    // give memory management to imagePub_
    imagePub_.publish(std::move(updated_img));
    // start a new image
    startNewImage();
  }
}

void Viewer::startNewImage()
{
  std::unique_ptr<sensor_msgs::Image> img(new sensor_msgs::Image(imageMsgTemplate_));
  img->data.resize(img->height * img->step, 0);  // allocate memory and set all bytes to zero
  imageUpdater_.setImage(&img);                  // event publisher will also render image now
}

void Viewer::eventMsg(const EventArray::ConstPtr & msg)
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
  auto decoder = decoderFactory_.getInstance(msg->encoding);
  if (!decoder) {
    ROS_INFO_STREAM("invalid encoding: " << msg->encoding);
    return;
  }
  // decode will produce callbacks to imageUpdater_
  decoder->setTimeBase(msg->time_base);
  decoder->decode(&(msg->events[0]), msg->events.size(), &imageUpdater_);
}

}  // namespace event_array_viewer
