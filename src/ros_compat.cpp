// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include "event_camera_renderer/ros_compat.h"
namespace event_camera_renderer
{
namespace ros_compat
{
#ifdef USING_ROS_1
//
// ------------------ ROS1 ---------------------
//
Time now() { return (ros::Time::now()); }
Time time_from_sec(const double sec) { return (Time(sec)); }

uint64_t to_nanoseconds(const Time & t) { return (t.toNSec()); }
Duration duration_from_nanoseconds(uint64_t nsec) { return (Duration().fromNSec(nsec)); }

#else
//
// ------------------ ROS2 ---------------------
//

Time now() { return (rclcpp::Clock().now()); }
Time time_from_sec(const double sec) { return (Time(static_cast<uint64_t>(sec * 1e9))); }
uint64_t to_nanoseconds(const Time & t) { return (t.nanoseconds()); }
Duration duration_from_nanoseconds(uint64_t nsec) { return (Duration::from_nanoseconds(nsec)); }
#endif
}  // namespace ros_compat
}  // namespace event_camera_renderer
