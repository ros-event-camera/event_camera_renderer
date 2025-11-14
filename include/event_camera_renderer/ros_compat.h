// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef EVENT_CAMERA_RENDERER__ROS_COMPAT_H_
#define EVENT_CAMERA_RENDERER__ROS_COMPAT_H_

#ifdef USING_ROS_1
#include <event_camera_msgs/EventPacket.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#else
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace event_camera_renderer
{
namespace ros_compat
{
#ifdef USING_ROS_1
//
// ------------------ ROS1 ---------------------
//
typedef std::unique_ptr<sensor_msgs::Image> ImgPtr;
using Time = ros::Time;
using Duration = ros::Duration;
using Writer = rosbag::Bag;
Time now();
Time time_from_sec(const double sec);

uint64_t to_nanoseconds(const Time & t);
Duration duration_from_nanoseconds(uint64_t nsec);

#else
//
// ------------------ ROS2 ---------------------
//
using ImgPtr = sensor_msgs::msg::Image::UniquePtr;
using Time = rclcpp::Time;
using Duration = rclcpp::Duration;
using EventPacket = event_camera_msgs::msg::EventPacket;

Time now();
Time time_from_sec(const double sec);
uint64_t to_nanoseconds(const Time & t);
Duration duration_from_nanoseconds(uint64_t nsec);
#endif
}  // namespace ros_compat
}  // namespace event_camera_renderer
#endif  // EVENT_CAMERA_RENDERER__ROS_COMPAT_H_
