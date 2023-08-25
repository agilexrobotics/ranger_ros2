/**
 * @file kinematics_model.hpp
 * @date 25-05-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
 */

#ifndef RANGER_BASE_KINEMATICS_MODEL_HPP
#define RANGER_BASE_KINEMATICS_MODEL_HPP

#include "boost/numeric/odeint.hpp"

namespace westonrobot {
template <typename Model>
class MotionModel {
 public:
  using State = typename Model::state_type;
  using Command = typename Model::control_type;

 public:
  State StepForward(State x0, Command u, double t0, double tf, double dt) {
    State x = x0;
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<typename Model::state_type>(),
        Model(u), x, t0, tf, dt);
    return x;
  }
};

class DualAckermanModel {
 public:
  using state_type = std::vector<double>;

  struct control_type {
    double v;
    double phi;
  };

 public:
  DualAckermanModel(double L, control_type u) : L_(L), u_(u){};

  // x1 = x, x2 = y, x3 = theta
  void operator()(const state_type& x, state_type& xd, double) {
    xd[0] = u_.v * std::cos(u_.phi) * std::cos(x[2]);
    xd[1] = u_.v * std::cos(u_.phi) * std::sin(x[2]);
    xd[2] = 2 * u_.v * std::sin(u_.phi) / L_;
  }

 private:
  double L_;
  control_type u_;
};

class ParallelModel {
 public:
  using state_type = std::vector<double>;

  struct control_type {
    double v;
    double phi;
  };

 public:
  ParallelModel(control_type u) : u_(u){};

  // x1 = x, x2 = y, x3 = theta
  void operator()(const state_type& x, state_type& xd, double) {
    xd[0] = u_.v * std::cos(x[2] + u_.phi);
    xd[1] = u_.v * std::sin(x[2] + u_.phi);
    xd[2] = 0;
  }

 private:
  control_type u_;
};

class SpinningModel {
 public:
  using state_type = std::vector<double>;

  struct control_type {
    double w;
  };

 public:
  SpinningModel(control_type u) : u_(u){};

  // x1 = x, x2 = y, x3 = theta
  void operator()(const state_type& x, state_type& xd, double) {
    xd[0] = 0;
    xd[1] = 0;
    xd[2] = u_.w;
  }

 private:
  control_type u_;
};
}  // namespace westonrobot

#endif /* RANGER_BASE_KINEMATICS_MODEL_HPP */
