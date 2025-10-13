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
#ifndef MIR_DWB_CRITICS_PATH_PROGRESS_H_
#define MIR_DWB_CRITICS_PATH_PROGRESS_H_

#include <dwb_critics/map_grid.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <string>
#include <vector>

namespace mir_dwb_critics
{
/**
 * @class PathProgressCritic
 * @brief Calculates an intermediate goal along the global path and scores trajectories on the distance to that goal.
 *
 * This trajectory critic helps ensure progress along the global path. It
 * calculates an intermediate goal that is as far as possible along the global
 * path as long as the path continues to move in one direction (+/-
 * angle_threshold).
 */
class PathProgressCritic : public dwb_critics::MapGridCritic
{
public:
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel, const geometry_msgs::Pose2D& goal,
               const nav_2d_msgs::Path2D& global_plan) override;

  void onInit() override;
  void reset() override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;

protected:
  bool getGoalPose(const geometry_msgs::Pose2D& robot_pose, const nav_2d_msgs::Path2D& global_plan, unsigned int& x,
                   unsigned int& y, double& desired_angle);

  unsigned int getGoalIndex(const std::vector<geometry_msgs::Pose2D>& plan, unsigned int start_index,
                            unsigned int last_valid_index, double& desired_angle, bool& has_forward_direction) const;

  bool findNextArticulation(const std::vector<geometry_msgs::Pose2D>& plan, unsigned int start_index,
                            unsigned int end_index, unsigned int last_valid_index, unsigned int& articulation_index,
                            double& articulation_yaw, bool& has_forward_direction) const;

  bool computeOutgoingAngle(const std::vector<geometry_msgs::Pose2D>& plan, unsigned int index,
                            double& angle) const;

  bool isPoseReached(const geometry_msgs::Pose2D& robot_pose, const geometry_msgs::Pose2D& goal_pose,
                     double goal_yaw, double xy_tolerance, double yaw_tolerance) const;

  bool isGoalReached(const geometry_msgs::Pose2D& robot_pose, const geometry_msgs::Pose2D& goal_pose,
                     double goal_yaw) const;

  double xy_local_goal_tolerance_;
  double yaw_local_goal_tolerance_;
  double final_goal_yaw_tolerance_;
  double final_goal_xy_tolerance_;
  double angle_threshold_;
  double articulation_angle_threshold_;
  double heading_scale_;
  bool enforce_forward_dot_;
  bool always_target_articulations_;
  bool initial_alignment_done_;

  // Plan change detection to reset internal state on new global plans
  bool have_last_plan_;
  size_t last_plan_size_;
  geometry_msgs::Pose2D last_plan_end_pose_;
  std::string last_plan_frame_id_;
  ros::Time last_plan_stamp_;
  std::vector<geometry_msgs::Pose2D> last_plan_;

  double plan_position_epsilon_;
  double plan_yaw_epsilon_;
  double plan_alignment_position_tolerance_;
  double plan_alignment_yaw_tolerance_;

  unsigned int last_progress_index_;
  bool holding_goal_;
  unsigned int held_goal_index_;
  geometry_msgs::Pose2D held_goal_pose_;
  double hold_position_epsilon_;
  double hold_yaw_epsilon_;

  std::vector<geometry_msgs::Pose2D> reached_intermediate_goals_;
  double desired_angle_;
  ros::Publisher intermediate_goal_pub_;
  ros::Publisher articulation_points_pub_;
  ros::Publisher intermediate_goal_tolerance_pub_;
};

}  // namespace mir_dwb_critics
#endif  // MIR_DWB_CRITICS_PATH_PROGRESS_H_
