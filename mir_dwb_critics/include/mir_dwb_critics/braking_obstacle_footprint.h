/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, DFKI GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MIR_DWB_CRITICS_BRAKING_OBSTACLE_FOOTPRINT_H_
#define MIR_DWB_CRITICS_BRAKING_OBSTACLE_FOOTPRINT_H_

#include <dwb_critics/obstacle_footprint.h>

namespace mir_dwb_critics
{
/**
 * @class BrakingObstacleFootprintCritic
 * @brief ObstacleFootprint that only requires the part of the rollout the robot needs to come to a stop to be free.
 *
 * Trajectory generators roll out sim_time seconds at (roughly) the sampled velocity. When the robot brakes towards a
 * stop point next to an obstacle, every such rollout reaches into the obstacle although the robot itself never
 * will, and the stock critic then rejects all trajectories. This critic checks the footprint only over the braking
 * distance from the sampled velocity at the deceleration limits (plus a reaction time), which is the invariant
 * that actually keeps it safe: it can always still stop before the obstacle.
 *
 * Optionally the checked footprint is inset by a couple of centimeters (obstacle cells are 5 cm squares marking a
 * laser hit somewhere inside, so an outline overlapping one by a centimeter is discretization, not contact) and a
 * current pose that already touches an obstacle does not make every trajectory illegal, which would otherwise
 * escalate into replanning from an invalid start pose and recovery behaviours.
 */
class BrakingObstacleFootprintCritic : public dwb_critics::ObstacleFootprintCritic
{
public:
  void onInit() override;
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel, const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;

  bool currentPoseBlocked() const
  {
    return current_pose_blocked_;
  }

protected:
  double decel_;
  double rot_decel_;
  double reaction_time_;
  double footprint_inset_;         ///< shrink the checked footprint by this much (m); costmap cells are 5 cm
  bool escape_current_collision_;  ///< keep scoring trajectories when the footprint already touches an obstacle
  bool current_pose_blocked_ = false;
};

}  // namespace mir_dwb_critics
#endif  // MIR_DWB_CRITICS_BRAKING_OBSTACLE_FOOTPRINT_H_
