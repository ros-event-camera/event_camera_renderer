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

#ifndef EVENT_ARRAY_VIEWER__VIEWER_ROS1_H_
#define EVENT_ARRAY_VIEWER__VIEWER_ROS1_H_

#include <event_array_codecs/decoder.h>
#include <event_array_codecs/decoder_factory.h>
#include <event_array_msgs/EventArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>

#include "event_array_viewer/display.h"

namespace event_array_viewer
{
class Viewer
{
public:
  using EventArray = event_array_msgs::EventArray;
  explicit Viewer(ros::NodeHandle & nh);
  ~Viewer();

private:
  void frameTimerExpired(const ros::TimerEvent &);
  void eventMsg(const EventArray::ConstPtr & msg);
  void imageConnectCallback(const image_transport::SingleSubscriberPublisher &);
  void startNewImage();

  // ------------------------  variables ------------------------------
  ros::NodeHandle nh_;
  std::shared_ptr<Display> display_;
  ros::Timer frameTimer_;     // fires once per frame
  double sliceTime_;          // duration of one frame
  ros::Subscriber eventSub_;  // subscribes to events
  bool isSubscribedToEvents_{false};
  image_transport::Publisher imagePub_;
  sensor_msgs::Image imageMsgTemplate_;
};
}  // namespace event_array_viewer
#endif  // EVENT_ARRAY_VIEWER__VIEWER_ROS1_H_
