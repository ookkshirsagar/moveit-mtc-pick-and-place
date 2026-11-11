#pragma once

#include <vector>
#include <string>

namespace ur5e_rh56dfx_manipulation
{

// prefix: "left_" or "right_" (matches the hand xacro prefix)
inline std::vector<std::string> getHandLinks(const std::string& prefix)
{
  std::vector<std::string> links = {
      // Wrist / base
      "wrist_base_link",
      "wrist_yaw",
      "hand_base",
      "palm",

      // Thumb
      "thumb_1",
      "thumb_2",
      "thumb_3",
      "thumb_4",
      "thumb_rubber_1",
      "thumb_rubber_2",
      "thumb_rubber_3",
      "thumb_tip",
      "thumb_pad",

      // Index
      "index_1",
      "index_2",
      "index_rubber_1",
      "index_rubber_2",
      "index_tip",
      "index_pad",

      // Middle
      "middle_1",
      "middle_2",
      "middle_rubber_1",
      "middle_rubber_2",
      "middle_tip",
      "middle_pad",

      // Ring
      "ring_1",
      "ring_2",
      "ring_rubber_1",
      "ring_rubber_2",
      "ring_tip",
      "ring_pad",

      // Little
      "little_1",
      "little_2",
      "little_rubber_1",
      "little_rubber_2",
      "little_tip",
      "little_pad",
  };
  for (auto& l : links) l = prefix + l;
  return links;
}

}  // namespace