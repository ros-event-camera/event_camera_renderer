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

#ifndef EVENT_ARRAY_VIEWER__CHECK_ENDIAN_H_
#define EVENT_ARRAY_VIEWER__CHECK_ENDIAN_H_

namespace event_array_viewer
{
namespace check_endian
{
// check endianness
inline bool isBigEndian()
{
  const union {
    uint32_t i;
    char c[4];
  } combined_int = {0x01020304};  // from stackoverflow
  return (combined_int.c[0] == 1);
}
}  // namespace check_endian
}  // namespace event_array_viewer
#endif  // EVENT_ARRAY_VIEWER__CHECK_ENDIAN_H_
