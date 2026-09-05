#pragma once

#include <array>
#include <cmath>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace omni_mppi_critics
{

/**
 * SteeringChangeCritic — 3-wheel steer+drive omni-base specific critic.
 *
 * Penalises trajectories where the physical steer angles of the three wheels
 * change rapidly between consecutive time steps.  This discourages the MPPI
 * sampler from picking trajectories that look kinematically cheap in (vx,vy,wz)
 * space but require large, slow steering movements in practice.
 *
 * Kinematics: mirrors omni_kinematics.py (omni_base_driver / robot_sim package).
 *   WHEEL_DIST = [0.32330, 0.32330, 0.38500]  m
 *   WHEEL_PHI  = [-0.883978, 0.883978, pi]    rad
 *   R_X[i]    = WHEEL_DIST[i] * cos(WHEEL_PHI[i])
 *   R_Y[i]    = WHEEL_DIST[i] * sin(WHEEL_PHI[i])
 *
 * For each wheel i:
 *   vx_i = vx - R_Y[i] * wz
 *   vy_i = vy + R_X[i] * wz
 *   phi_i = atan2(vy_i, vx_i)
 *   (flipped by ±pi if |phi_i| > 0.71*pi  — same flip logic as real driver)
 *
 * Cost per trajectory = sum over wheels and time steps of |Δphi_i(t)|.
 *
 * Parameters (all under the critic's namespace in nav2_params.yaml):
 *   enabled    bool   Enable/disable critic (default true)
 *   cost_power int    Exponent on raw cost (default 1)
 *   cost_weight float Scale factor (default 2.5)
 */
class SteeringChangeCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

private:
  int cost_power_{1};
  float cost_weight_{2.5f};

  // Pre-computed wheel geometry (matches omni_kinematics.py)
  static constexpr int N_WHEELS = 3;
  static constexpr float FLIP_THRESHOLD = 0.71f * static_cast<float>(M_PI);

  // R_X[i] = WHEEL_DIST[i] * cos(WHEEL_PHI[i])
  // R_Y[i] = WHEEL_DIST[i] * sin(WHEEL_PHI[i])
  static constexpr std::array<float, N_WHEELS> R_X{
     0.20633f,   // 0.32330 * cos(-0.883978)
     0.20633f,   // 0.32330 * cos( 0.883978)
    -0.38500f,   // 0.38500 * cos(pi)
  };
  static constexpr std::array<float, N_WHEELS> R_Y{
    -0.24897f,   // 0.32330 * sin(-0.883978)
     0.24897f,   // 0.32330 * sin( 0.883978)
     0.00000f,   // 0.38500 * sin(pi) ≈ 0
  };
};

}  // namespace omni_mppi_critics
