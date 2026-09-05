#include "omni_mppi_critics/steering_change_critic.hpp"

#include <cmath>

#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xtensor.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace omni_mppi_critics
{

void SteeringChangeCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(enabled_,      "enabled",      true);
  getParam(cost_power_,   "cost_power",   1);
  getParam(cost_weight_,  "cost_weight",  2.5f);

  RCLCPP_INFO(
    logger_,
    "SteeringChangeCritic: weight=%.2f  power=%d  wheels=%d",
    cost_weight_, cost_power_, N_WHEELS);
}

void SteeringChangeCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // state.vx/vy/wz: shape [batch_size, time_steps]
  const auto & vx = data.state.vx;
  const auto & vy = data.state.vy;
  const auto & wz = data.state.wz;

  const std::size_t B = vx.shape(0);
  const std::size_t T = vx.shape(1);

  if (T < 2) {
    return;
  }

  // Accumulate steering-change cost per batch
  xt::xtensor<float, 1> batch_cost = xt::zeros<float>({B});

  for (int w = 0; w < N_WHEELS; ++w) {
    // Wheel velocities: [batch, time]
    xt::xtensor<float, 2> vx_w = vx - R_Y[w] * wz;   // vx - R_Y[w]*wz
    xt::xtensor<float, 2> vy_w = vy + R_X[w] * wz;   // vy + R_X[w]*wz

    // Steer angle: [batch, time]
    xt::xtensor<float, 2> phi = xt::atan2(vy_w, vx_w);

    // Apply flip logic (mirrors omni_kinematics.py)
    phi = xt::where(phi >  FLIP_THRESHOLD, phi - static_cast<float>(M_PI), phi);
    phi = xt::where(phi < -FLIP_THRESHOLD, phi + static_cast<float>(M_PI), phi);

    // Delta phi between consecutive steps: [batch, time-1]
    // xt::diff along axis 1
    auto phi_t1 = xt::view(phi, xt::all(), xt::range(1, xt::placeholders::_));  // t=1..T-1
    auto phi_t0 = xt::view(phi, xt::all(), xt::range(0, T - 1));                // t=0..T-2
    xt::xtensor<float, 2> dphi = xt::abs(phi_t1 - phi_t0);

    // Sum over time steps → [batch]
    batch_cost += xt::sum(dphi, {1});
  }

  // Apply power and weight, add to costs
  if (cost_power_ == 1) {
    data.costs += cost_weight_ * batch_cost;
  } else {
    data.costs += cost_weight_ * xt::pow(batch_cost, static_cast<float>(cost_power_));
  }
}

}  // namespace omni_mppi_critics

PLUGINLIB_EXPORT_CLASS(
  omni_mppi_critics::SteeringChangeCritic,
  mppi::critics::CriticFunction)
