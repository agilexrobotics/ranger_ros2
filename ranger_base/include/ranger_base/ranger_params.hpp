/**
 * @Kit       : VS-CODE  Desktop
 * @Author    : TX
 * @Date      : 2022-02-23  11:41:28
 * @FileName  : ranger_params.hpp
 * Copyright  : AgileX Robotics (2022)
 **/
 

#ifndef RANGER_PARAMS_HPP
#define RANGER_PARAMS_HPP

#include <cstdint>

namespace westonrobot {
/* ranger Parameters */
struct RangerParams {
  static constexpr double track =
      0.36;  // in meter (left & right wheel distance)
  static constexpr double wheelbase =
      0.36;  // in meter (front & rear wheel distance)

  static constexpr double max_linear_speed = 1.5;      // in m/s
  static constexpr double max_angular_speed = 0.7853;  // in rad/s
  static constexpr double max_speed_cmd = 10.0;        // in rad/s

  static constexpr double max_steer_angle_central = 0.53385; //~= 30.58 degree
  static constexpr double max_steer_angle_slide = 0.69813; // 40 degree
};

}  // namespace westonrobot

#endif /* RANGER_PARAMS_HPP */
