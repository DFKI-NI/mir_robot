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
#ifndef MIR_DWB_CRITICS_PATH_FOLLOWER_H_
#define MIR_DWB_CRITICS_PATH_FOLLOWER_H_

#include <dwb_local_planner/trajectory_critic.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/publisher.h>
#include <cstddef>
#include <vector>

namespace mir_dwb_critics
{
/**
 * @class PathFollowerCritic
 * @brief Follows the global plan with a designed velocity profile instead of chasing waypoints.
 *
 * The plan is split into motion segments separated by "stop points": in-place turns of at least
 * stop_turn_angle, direction reversals and the final pose. Between stop points the robot cruises at
 * max speed and only decelerates (sqrt profile, then proportional) into the next stop point. At a
 * stop point it rotates in place with a trapezoidal angular velocity profile and continues. Small
 * in-place turns in the plan are replaced by fillets and driven through, and the speed is limited by the
 * curvature of the path ahead so that the available angular velocity suffices to follow it.
 *
 * Raw score of a trajectory (DWB multiplies the sum by `scale`):
 *  - speed:    speed_scale * ((v - v_desired) / v_max)^2
 *  - rotation: rotation_scale * ((w - w_desired) / w_max)^2      (while turning at a stop point)
 *  - lateral:  path_distance_scale * (endpoint deviation + corner cutting excess) / resolution
 *  - heading:  heading_scale * (endpoint heading - path heading)^2  (while driving)
 */
class PathFollowerCritic : public dwb_local_planner::TrajectoryCritic
{
public:
  struct Vertex
  {
    double x = 0.0;
    double y = 0.0;
    double theta_in = 0.0;   ///< heading of the plan when arriving at this vertex
    double theta_out = 0.0;  ///< heading of the plan when leaving this vertex
    double s = 0.0;          ///< arc length from the first vertex
    bool forward = true;     ///< the segment leaving this vertex is driven forwards
    bool stop = false;       ///< the robot has to stop here (turn, reversal or final pose)
  };

  void onInit() override;
  void reset() override;
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel, const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;

  /// Split a plan into vertices with stop points and fillet the driven-through turns (exposed for tests).
  std::vector<Vertex> buildPolyline(const nav_2d_msgs::Path2D& plan, const geometry_msgs::Pose2D& goal) const;
  std::vector<Vertex> filletCorners(const std::vector<Vertex>& vertices) const;
  double desiredSpeed(double distance_to_stop) const;
  double desiredRotation(double heading_error) const;

  bool turning() const
  {
    return turning_;
  }
  double desiredVelocity() const
  {
    return desired_speed_;
  }
  geometry_msgs::Pose2D target() const
  {
    return target_;
  }

protected:
  struct Projection
  {
    size_t segment = 0;  ///< index of the vertex starting the segment
    double t = 0.0;      ///< position within the segment [0, 1] (outside when the segment was extended)
    double distance = 0.0;
    double s = 0.0;  ///< arc length
    double x = 0.0;
    double y = 0.0;
  };

  Projection project(double x, double y, size_t first_segment, size_t last_segment, bool extend_ends = false) const;
  bool locateRobot(const geometry_msgs::Pose2D& pose, bool passed_valid, size_t passed_index);
  bool findStop(const geometry_msgs::Pose2D& stop, size_t& index) const;
  static bool isContinuationOf(const std::vector<geometry_msgs::Pose2D>& plan,
                               const std::vector<geometry_msgs::Pose2D>& previous);
  double pathHeading(const Projection& projection) const;
  void publishTarget(const nav_2d_msgs::Path2D& plan) const;

  // parameters
  double xy_local_goal_tolerance_;
  double xy_final_goal_tolerance_;
  double yaw_local_goal_tolerance_;
  double stop_turn_angle_;
  double start_turn_angle_;
  double max_speed_;
  double max_rotation_;
  double decel_;
  double rot_decel_;
  double approach_gain_;
  double rotation_gain_;
  double speed_scale_;
  double rotation_scale_;
  double heading_scale_;
  double path_distance_scale_;
  double search_window_;
  double corner_fillet_length_;
  double curve_rotation_;
  double curve_lookahead_;
  double plan_alignment_position_tolerance_;

  // per-cycle context
  std::vector<Vertex> vertices_;
  Projection robot_;
  size_t window_first_ = 0;  ///< first segment used for scoring deviations
  size_t window_last_ = 0;   ///< last segment used for scoring deviations
  bool turning_ = false;
  double desired_speed_ = 0.0;
  double desired_rotation_ = 0.0;
  double speed_direction_ = 1.0;
  geometry_msgs::Pose2D target_;
  bool prepared_ = false;

  // state across cycles
  std::vector<geometry_msgs::Pose2D> last_plan_;
  bool have_progress_ = false;
  double progress_x_ = 0.0;
  double progress_y_ = 0.0;
  size_t progress_segment_ = 0;
  bool have_passed_stop_ = false;
  geometry_msgs::Pose2D passed_stop_;
  bool have_turning_stop_ = false;
  geometry_msgs::Pose2D turning_stop_;

  ros::Publisher intermediate_goal_pub_;
};

}  // namespace mir_dwb_critics
#endif  // MIR_DWB_CRITICS_PATH_FOLLOWER_H_
