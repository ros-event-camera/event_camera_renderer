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

#ifndef EVENT_CAMERA_RENDERER__SHARP_UPDATER_H_
#define EVENT_CAMERA_RENDERER__SHARP_UPDATER_H_

#include <memory.h>

#include <memory>
#include <queue>
#ifdef USING_ROS_1
#include <sensor_msgs/Image.h>
typedef std::unique_ptr<sensor_msgs::Image> ImgPtr;
#else
#include <sensor_msgs/msg/image.hpp>
using ImgPtr = sensor_msgs::msg::Image::UniquePtr;
#endif

#include <event_camera_codecs/event_processor.h>

namespace event_camera_renderer
{
class SharpUpdater : public event_camera_codecs::EventProcessor
{
public:
  // ---------- inherited from EventProcessor
  inline void eventCD(uint64_t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    const uint32_t offset = stride_ * ey + ex * 3 + (polarity ? 0 : 2);
    if (sharpImg_[offset] != 0) {
      // drop duplicate events of the same polarity at the same pixel
      return;
    }
    sharpImg_[offset] = 255;
    numOccupiedPixels_++;
    events_.push(Event(ex, ey, polarity));
    const size_t blockIdx = getBlockIdx(ex, ey);
    if (sharpImg_[blockIdx] == 0) {
      numOccupiedBlocks_++;
    }
    sharpImg_[blockIdx] += 1;  // bump counter of this block
    updateEventWindowSize();
  }

  void setBlockSize(uint16_t b)
  {
    if (b > 11) {  // sqrt(127)
      std::cerr << "max block size is 11" << std::endl;
      throw std::runtime_error("block size too large!");
    }
    blockSize_ = b;
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}

  // clang-format off
  void finished() override{};
  void rawData(const char *, size_t) override{};
  // clang-format on
  // --------- end of inherited from EventProcessor

  // note: returns reference to pointer to allow for std::move()
  inline ImgPtr getImage()
  {
    // keep state by copying sharp image
    memcpy(&img_->data[0], &sharpImg_[0], img_->data.size());
    // zero out the green value, leave
    for (size_t i = 1; i < img_->data.size(); i += 3) {
      img_->data[i] = 0;
    }
    return (std::move(img_));
  }
  // check if currently building an image
  inline bool hasImage() const { return (img_ != nullptr); }

  // deallocate memory associated with pointer
  inline void resetImagePtr() { img_.reset(); }

  // take ownership of pointer and memory
  inline void setImage(ImgPtr * img)
  {
    img_ = std::move(*img);
    if (
      sharpImg_.empty() || width_ != img_->width || height_ != img_->height ||
      stride_ != img_->step) {
      // first call, allocate memory
      sharpImg_.clear();
      sharpImg_.resize(img_->data.size(), 0);
      width_ = img_->width;
      height_ = img_->height;
      stride_ = img_->step;
      blockStrideX_ = blockSize_ * 3;
      blockStrideY_ = width_ * blockSize_ * 3;
    }
  }

private:
  struct Event
  {
    Event(uint16_t ex, uint16_t ey, uint8_t polarity) : x(ex), y(ey), p(polarity) {}
    uint16_t x;
    uint16_t y;
    uint8_t p;
  };

  void updateEventWindowSize()
  {
    while (events_.size() > eventWindowSize_) {
      const Event & e = events_.front();
      const uint32_t offset = stride_ * e.y + e.x * 3 + (e.p ? 0 : 2);
      (void)offset;
      sharpImg_[offset] = 0;
      numOccupiedPixels_--;

      const size_t blockIdx = getBlockIdx(e.x, e.y);
      sharpImg_[blockIdx]--;
      if (sharpImg_[blockIdx] == 0) {
        numOccupiedBlocks_--;
      }
      events_.pop();  // remove element now
    }
    // adjust event window size up or down to match the fill ratio:
    // new_size = old_size * current_fill_ratio / desired_fill_ratio
    // The idea is that as the event window increases, the features will "fill out"
    eventWindowSize_ = (eventWindowSize_ * numOccupiedBlocks_ * fillRatioDenom_) /
                       (numOccupiedPixels_ * fillRatioNum_);
  }

  inline size_t getBlockIdx(uint16_t ex, uint16_t ey) const
  {
    // the block data is stored in the green value (+1)
    // of the top left corner of each block
    return ((ey / blockSize_) * blockStrideY_ + (ex / blockSize_) * blockStrideX_ + 1);
  }
  // ----------------- variables -------
  ImgPtr img_;
  size_t stride_{0};               // stride for whole image (in bytes)
  size_t width_{0};                // whole image width
  size_t height_{0};               // whole image height
  std::vector<uint8_t> sharpImg_;  // the sharp image
  uint16_t blockSize_{2};          // edge length of one block, in pixels
  uint16_t blockStrideX_{0};       // stride for block access in X
  uint16_t blockStrideY_{0};       // stride for block access in Y
  size_t eventWindowSize_{2000};   // event window size
  size_t fillRatioDenom_{2};       // denominator of fill ratio
  size_t fillRatioNum_{1};         // numerator of fill ratio
  std::queue<Event> events_;       // queue with buffered events
  uint32_t numOccupiedPixels_{0};  // currently occupied number of pixels
  uint32_t numOccupiedBlocks_{0};  // currently occupied number of blocks
};
}  // namespace event_camera_renderer
#endif  // EVENT_CAMERA_RENDERER__SHARP_UPDATER_H_
