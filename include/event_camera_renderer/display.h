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

#ifndef EVENT_CAMERA_RENDERER__DISPLAY_H_
#define EVENT_CAMERA_RENDERER__DISPLAY_H_

#include <event_camera_renderer/ros_compat.h>

#include <memory>

namespace event_camera_renderer
{
class Display
{
public:
  virtual ~Display() {}
  //
  // ------------- pure virtual functions
  //
  virtual void initialize(const ros_compat::EventPacket & msg) = 0;

  virtual bool update(
    const ros_compat::EventPacket & msg, uint64_t timeLimit, uint64_t * nextTime) = 0;

  virtual ros_compat::ImgPtr getImage() = 0;

  virtual void setImage(ros_compat::ImgPtr * img) = 0;  // take ownership of pointer and memory

  //
  // ------------- end of pure virtual functions
  //

  inline bool isInitialized() const { return (t0_ != std::numeric_limits<int64_t>::lowest()); }
  inline bool setHeaderTime(const ros_compat::Time & t)
  {
    if (isInitialized() && t < rosHeaderTime_) {
      rosHeaderTime_ = t;
      return (false);
    }
    rosHeaderTime_ = t;
    return (true);
  }
  inline void setIsFirstTimeInPacket(bool b) { isFirstTimeInPacket_ = b; }

  // check if currently building an image
  inline bool hasImage() const { return (img_ != nullptr); }

  // deallocate memory associated with pointer
  inline void resetImagePtr() { img_.reset(); }

  inline uint64_t rosToSensorTime(const rclcpp::Time & t) const { return (t.nanoseconds() - t0_); }
  inline rclcpp::Time sensorToRosTime(uint64_t t) const
  {
    return (rclcpp::Time(t + t0_, RCL_ROS_TIME));
  }

  inline void maybeUpdateRosToSensorTimeOffset(uint64_t sensor_time)
  {
    if (isFirstTimeInPacket_) {
      updateRosToSensorTimeOffset(rosHeaderTime_, static_cast<int64_t>(sensor_time));
      isFirstTimeInPacket_ = false;
    }
  }
  void updateRosToSensorTimeOffset(const ros_compat::Time & t_ros, int64_t t_sens)
  {
    if (t0_ == std::numeric_limits<int64_t>::lowest()) {
      t0_ = static_cast<int64_t>(t_ros.nanoseconds()) - t_sens;
      t0_init_ = t0_;
    } else {
      // to avoid rounding errors, first subtract off the large t0_init_,
      // which contains the time since epoch.
      const double dt_meas = static_cast<double>(
        static_cast<int64_t>(t_ros.nanoseconds()) - t0_init_ - t_sens);  // measured
      const double dt_k = static_cast<double>(t0_ - t0_init_);           // current estimate
      constexpr double alpha = 1.0 / 100.0;
      t0_ = t0_init_ + static_cast<int64_t>(dt_k * (1 - alpha) + dt_meas * alpha);
    }
  }

  void resetTime()
  {
    isFirstTimeInPacket_ = true;
    t0_ = std::numeric_limits<int64_t>::lowest();  // ros to sensor time offset
    t0_init_ = 0;
  }

  // -----------------
  // factory method
  static std::shared_ptr<Display> newInstance(const std::string & type);

protected:
  Display() {}
  ros_compat::ImgPtr img_;
  bool isFirstTimeInPacket_{true};
  ros_compat::Time rosHeaderTime_{0LL, RCL_ROS_TIME};
  int64_t t0_{std::numeric_limits<int64_t>::lowest()};  // ros to sensor time offset
  int64_t t0_init_{0};
};
}  // namespace event_camera_renderer
#endif  // EVENT_CAMERA_RENDERER__DISPLAY_H_
