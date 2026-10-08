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
#include <mir_dwb_critics/braking_obstacle_footprint.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <algorithm>
#include <cmath>

namespace mir_dwb_critics
{
void BrakingObstacleFootprintCritic::onInit()
{
  dwb_critics::ObstacleFootprintCritic::onInit();
  double acc_lim_x = 1.5, acc_lim_theta = 2.0, decel_lim_x, decel_lim_theta;
  planner_nh_.param("acc_lim_x", acc_lim_x, acc_lim_x);
  planner_nh_.param("acc_lim_theta", acc_lim_theta, acc_lim_theta);
  planner_nh_.param("decel_lim_x", decel_lim_x, -acc_lim_x);
  planner_nh_.param("decel_lim_theta", decel_lim_theta, -acc_lim_theta);
  critic_nh_.param("decel", decel_, std::fabs(decel_lim_x));
  critic_nh_.param("rot_decel", rot_decel_, std::fabs(decel_lim_theta));
  critic_nh_.param("reaction_time", reaction_time_, 0.1);
  critic_nh_.param("footprint_inset", footprint_inset_, 0.0);
  critic_nh_.param("escape_current_collision", escape_current_collision_, true);
  decel_ = std::max(decel_, 1e-3);
  rot_decel_ = std::max(rot_decel_, 1e-3);
  // Shrink the checked polygon towards its centre (exact for rectangles around the base frame).
  for (auto& point : footprint_spec_.points)
  {
    point.x -= std::copysign(std::min(footprint_inset_, std::fabs(point.x)), point.x);
    point.y -= std::copysign(std::min(footprint_inset_, std::fabs(point.y)), point.y);
  }
}

bool BrakingObstacleFootprintCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                             const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan)
{
  if (!dwb_critics::ObstacleFootprintCritic::prepare(pose, vel, goal, global_plan))
  {
    return false;
  }
  // If the footprint already touches an obstacle cell where the robot stands, nothing the controller does can
  // change that pose. Rejecting every trajectory for it would only trigger replanning from an "invalid" start and
  // recovery behaviours; instead keep judging trajectories by where they lead.
  current_pose_blocked_ = false;
  if (escape_current_collision_)
  {
    try
    {
      scorePose(*costmap_, pose);
    }
    catch (const nav_core2::IllegalTrajectoryException&)
    {
      current_pose_blocked_ = true;
      ROS_WARN_THROTTLE_NAMED(2.0, "BrakingObstacleFootprint",
                              "Footprint at the current pose touches an obstacle; scoring trajectories by where they lead.");
    }
  }
  return true;
}

double BrakingObstacleFootprintCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
  // The rollout runs at constant velocity, so the braking distance v^2 / (2 a) is covered after v / (2 a).
  double speed = std::hypot(traj.velocity.x, traj.velocity.y);
  double horizon =
      std::max(speed / (2.0 * decel_), std::fabs(traj.velocity.theta) / (2.0 * rot_decel_)) + reaction_time_;
  const nav_core2::Costmap& costmap = *costmap_;
  double score = 0.0;
  for (unsigned int i = 0; i < traj.poses.size(); ++i)
  {
    if (i > 0 && i < traj.time_offsets.size() && traj.time_offsets[i].toSec() > horizon)
    {
      break;
    }
    if (i == 0 && current_pose_blocked_)
    {
      continue;
    }
    double pose_score = scorePose(costmap, traj.poses[i]);
    score = static_cast<double>(sum_scores_) * score + pose_score;
  }
  return score;
}

}  // namespace mir_dwb_critics

PLUGINLIB_EXPORT_CLASS(mir_dwb_critics::BrakingObstacleFootprintCritic, dwb_local_planner::TrajectoryCritic)
