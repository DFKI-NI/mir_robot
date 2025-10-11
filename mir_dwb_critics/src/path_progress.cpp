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
#include <mir_dwb_critics/path_progress.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_grid/coordinate_conversion.h>
#include <pluginlib/class_list_macros.h>
#include <nav_2d_utils/path_ops.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace mir_dwb_critics
{
bool PathProgressCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan)
{
  dwb_critics::MapGridCritic::reset();

  unsigned int local_goal_x, local_goal_y;
  if (!getGoalPose(pose, global_plan, local_goal_x, local_goal_y, desired_angle_))
  {
    return false;
  }

  // Enqueue just the last pose
  cell_values_.setValue(local_goal_x, local_goal_y, 0.0);
  queue_->enqueueCell(local_goal_x, local_goal_y);

  propogateManhattanDistances();

  return true;
}

void PathProgressCritic::onInit()
{
  dwb_critics::MapGridCritic::onInit();
  critic_nh_.param("xy_local_goal_tolerance", xy_local_goal_tolerance_, 0.20);
  critic_nh_.param("yaw_local_goal_tolerance", yaw_local_goal_tolerance_, 0.15);
  ros::NodeHandle private_nh("~");
  if (!private_nh.getParam("yaw_goal_tolerance", final_goal_yaw_tolerance_))
  {
    critic_nh_.param("yaw_goal_tolerance", final_goal_yaw_tolerance_, yaw_local_goal_tolerance_);
  }
  critic_nh_.param("angle_threshold", angle_threshold_, M_PI_4);
  critic_nh_.param("articulation_angle_threshold", articulation_angle_threshold_, 1.3089969389957472);
  critic_nh_.param("heading_scale", heading_scale_, 1.0);
  critic_nh_.param("enforce_forward_dot", enforce_forward_dot_, true);

  intermediate_goal_pub_ = critic_nh_.advertise<geometry_msgs::PoseStamped>("intermediate_goal", 1);

  articulation_angle_threshold_ = std::max(articulation_angle_threshold_, angle_threshold_);

  // divide heading scale by position scale because the sum will be multiplied by scale again
  heading_scale_ /= getScale();
  last_progress_index_ = 0;
  reached_intermediate_goals_.clear();
  holding_goal_ = false;
  held_goal_index_ = 0;
  held_goal_pose_.x = 0.0;
  held_goal_pose_.y = 0.0;
  held_goal_pose_.theta = 0.0;
  hold_position_epsilon_ = 1e-6;
  hold_yaw_epsilon_ = 1e-6;
  final_goal_yaw_tolerance_ = std::max(final_goal_yaw_tolerance_, 1e-6);
}

void PathProgressCritic::reset()
{
  reached_intermediate_goals_.clear();
  last_progress_index_ = 0;
  holding_goal_ = false;
  held_goal_index_ = 0;
  held_goal_pose_.x = 0.0;
  held_goal_pose_.y = 0.0;
  held_goal_pose_.theta = 0.0;
}

double PathProgressCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
  double position_score = MapGridCritic::scoreTrajectory(traj);
  double heading_diff = fabs(angles::shortest_angular_distance(traj.poses.back().theta, desired_angle_));
  double heading_score = heading_diff * heading_diff;

  return position_score + heading_scale_ * heading_score;
}

bool PathProgressCritic::getGoalPose(const geometry_msgs::Pose2D& robot_pose, const nav_2d_msgs::Path2D& global_plan,
                                     unsigned int& x, unsigned int& y, double& desired_angle)
{
  const nav_core2::Costmap& costmap = *costmap_;
  const nav_grid::NavGridInfo& info = costmap.getInfo();

  if (global_plan.poses.empty())
  {
    ROS_ERROR_NAMED("PathProgressCritic", "The global plan was empty.");
    return false;
  }

  std::vector<geometry_msgs::Pose2D> plan = nav_2d_utils::adjustPlanResolution(global_plan, info.resolution).poses;

  if (plan.empty())
  {
    ROS_ERROR_NAMED("PathProgressCritic", "The adjusted global plan was empty.");
    return false;
  }

  if (holding_goal_ && held_goal_index_ >= plan.size())
  {
    ROS_DEBUG_NAMED("PathProgressCritic", "Held goal index %u is out of range for current plan of size %zu. Releasing hold.",
                    held_goal_index_, plan.size());
    holding_goal_ = false;
    held_goal_index_ = 0;
  }

  // find the "start pose", i.e. the pose on the plan closest to the robot that is also on the local map
  unsigned int start_index = 0;
  double distance_to_start = std::numeric_limits<double>::infinity();
  bool started_path = false;
  for (unsigned int i = 0; i < plan.size(); i++)
  {
    double g_x = plan[i].x;
    double g_y = plan[i].y;
    unsigned int map_x, map_y;
    if (worldToGridBounded(info, g_x, g_y, map_x, map_y) && costmap(map_x, map_y) != nav_core2::Costmap::NO_INFORMATION)
    {
      // Still on the costmap. Continue.
      double distance = nav_2d_utils::poseDistance(plan[i], robot_pose);
      if (distance_to_start > distance)
      {
        start_index = i;
        distance_to_start = distance;
        started_path = true;
      }
      else
      {
        // Plan is going away from the robot again. It's possible that it comes back and we would find a pose that's
        // even closer to the robot, but then we would skip over parts of the plan.
        break;
      }
    }
    else if (started_path)
    {
      // Off the costmap after being on the costmap.
      break;
    }
    // else, we have not yet found a point on the costmap, so we just continue
  }

  if (!started_path)
  {
    ROS_ERROR_NAMED("PathProgressCritic", "None of the points of the global plan were in the local costmap.");
    return false;
  }

  // find the "last valid pose", i.e. the last pose on the plan after the start pose that is still on the local map
  unsigned int last_valid_index = start_index;
  for (unsigned int i = start_index + 1; i < plan.size(); i++)
  {
    double g_x = plan[i].x;
    double g_y = plan[i].y;
    unsigned int map_x, map_y;
    if (worldToGridBounded(info, g_x, g_y, map_x, map_y) && costmap(map_x, map_y) != nav_core2::Costmap::NO_INFORMATION)
    {
      // Still on the costmap. Continue.
      last_valid_index = i;
    }
    else
    {
      // Off the costmap after being on the costmap.
      break;
    }
  }

  // Constrain the search range to enforce monotonic progress.
  unsigned int plan_last_index = static_cast<unsigned int>(plan.size() - 1);
  last_progress_index_ = std::min(last_progress_index_, plan_last_index);

  unsigned int search_start_index = std::max(start_index, last_progress_index_);
  search_start_index = std::min(search_start_index, last_valid_index);

  unsigned int articulation_scan_start = 0u;
  if (plan_last_index >= 1)
  {
    unsigned int next_index = last_progress_index_ < plan_last_index ? last_progress_index_ + 1 : plan_last_index;
    articulation_scan_start = std::max(next_index, 1u);
  }
  else
  {
    articulation_scan_start = plan_last_index;
  }

  auto publishIntermediateGoal = [&](const geometry_msgs::Pose2D& goal_pose, double goal_yaw) {
    if (!intermediate_goal_pub_)
    {
      return;
    }
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header = global_plan.header;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.pose.position.x = goal_pose.x;
    goal_msg.pose.position.y = goal_pose.y;
    goal_msg.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal_yaw);
    goal_msg.pose.orientation.x = q.x();
    goal_msg.pose.orientation.y = q.y();
    goal_msg.pose.orientation.z = q.z();
    goal_msg.pose.orientation.w = q.w();
    intermediate_goal_pub_.publish(goal_msg);
  };

  if (holding_goal_)
  {
    double yaw_error = fabs(angles::shortest_angular_distance(robot_pose.theta, held_goal_pose_.theta));
    if (yaw_error >= final_goal_yaw_tolerance_)
    {
      unsigned int held_x = 0;
      unsigned int held_y = 0;
      if (worldToGridBounded(info, held_goal_pose_.x, held_goal_pose_.y, held_x, held_y))
      {
        x = held_x;
        y = held_y;
        desired_angle = held_goal_pose_.theta;
        publishIntermediateGoal(held_goal_pose_, held_goal_pose_.theta);
        ROS_DEBUG_NAMED("PathProgressCritic", "Holding goal index %u due to yaw error %.3f rad (threshold %.3f)",
                        held_goal_index_, yaw_error, final_goal_yaw_tolerance_);
        return true;
      }

      ROS_WARN_NAMED("PathProgressCritic",
                     "Held goal (index %u) is outside the local costmap. Releasing hold to search for a new goal.",
                     held_goal_index_);
      holding_goal_ = false;
      held_goal_index_ = 0;
    }
  }

  if (holding_goal_ && held_goal_index_ >= last_progress_index_)
  {
    search_start_index = std::min(search_start_index, held_goal_index_);
  }

  unsigned int goal_index = search_start_index;
  double goal_yaw = plan[goal_index].theta;
  bool has_forward_direction = false;
  bool found_goal = false;

  unsigned int search_index = search_start_index;
  while (search_index <= last_valid_index)
  {
    double candidate_yaw = goal_yaw;
    bool candidate_has_forward = false;
    unsigned int candidate_index =
        getGoalIndex(plan, search_index, last_valid_index, candidate_yaw, candidate_has_forward);

    bool forced_articulation = false;
    if (candidate_index > last_progress_index_)
    {
      unsigned int articulation_index = 0;
      double articulation_yaw = candidate_yaw;
      bool articulation_has_forward = candidate_has_forward;
      if (findNextArticulation(plan, articulation_scan_start, candidate_index, last_valid_index,
                               articulation_index, articulation_yaw, articulation_has_forward))
      {
        candidate_index = articulation_index;
        candidate_yaw = articulation_yaw;
        candidate_has_forward = articulation_has_forward;
        forced_articulation = true;
      }
    }

    if (!forced_articulation)
    {
      candidate_index = std::max(candidate_index, search_index);
    }

    if (isGoalReached(robot_pose, plan[candidate_index], candidate_yaw))
    {
      last_progress_index_ = std::max(last_progress_index_, candidate_index);
      geometry_msgs::Pose2D reached_pose = plan[candidate_index];
      reached_pose.theta = candidate_yaw;
      if (reached_intermediate_goals_.empty() ||
          nav_2d_utils::poseDistance(reached_intermediate_goals_.back(), reached_pose) > 1e-6 ||
          fabs(angles::shortest_angular_distance(reached_intermediate_goals_.back().theta, reached_pose.theta)) > 1e-6)
      {
        reached_intermediate_goals_.push_back(reached_pose);
      }
      ROS_DEBUG_NAMED("PathProgressCritic",
                      "Reached intermediate goal index %u. last_progress_index_: %u", candidate_index,
                      last_progress_index_);

      if (candidate_index >= last_valid_index)
      {
        goal_index = candidate_index;
        goal_yaw = candidate_yaw;
        has_forward_direction = candidate_has_forward;
        found_goal = true;
        break;
      }

      search_index = candidate_index + 1;
      continue;
    }

    if (enforce_forward_dot_ && candidate_has_forward && !forced_articulation)
    {
      double to_goal_x = plan[candidate_index].x - robot_pose.x;
      double to_goal_y = plan[candidate_index].y - robot_pose.y;
      double dot = to_goal_x * std::cos(candidate_yaw) + to_goal_y * std::sin(candidate_yaw);
      if (dot < 0.0)
      {
        ROS_DEBUG_NAMED("PathProgressCritic",
                        "Skipping goal index %u due to backward alignment (dot product %.3f)", candidate_index, dot);
        if (candidate_index >= last_valid_index)
        {
          break;
        }
        search_index = candidate_index + 1;
        continue;
      }
    }

    goal_index = candidate_index;
    goal_yaw = candidate_yaw;
    has_forward_direction = candidate_has_forward;
    found_goal = true;
    break;
  }

  if (!found_goal)
  {
    unsigned int fallback_start = std::min(last_valid_index, search_start_index);
    goal_index = fallback_start;
    bool selected_fallback = false;

    for (; goal_index <= last_valid_index; ++goal_index)
    {
      has_forward_direction = computeOutgoingAngle(plan, goal_index, goal_yaw);
      if (has_forward_direction)
      {
        if (!enforce_forward_dot_)
        {
          selected_fallback = true;
          break;
        }

        double to_goal_x = plan[goal_index].x - robot_pose.x;
        double to_goal_y = plan[goal_index].y - robot_pose.y;
        double dot = to_goal_x * std::cos(goal_yaw) + to_goal_y * std::sin(goal_yaw);
        if (dot >= 0.0)
        {
          selected_fallback = true;
          break;
        }
        continue;
      }

      goal_yaw = plan[goal_index].theta;
      if (!enforce_forward_dot_ || goal_index == last_valid_index)
      {
        selected_fallback = true;
        break;
      }
    }

    if (!selected_fallback)
    {
      return false;
    }

    if (goal_index > last_progress_index_)
    {
      unsigned int articulation_index = 0;
      double articulation_yaw = goal_yaw;
      bool articulation_has_forward = has_forward_direction;
      if (findNextArticulation(plan, articulation_scan_start, goal_index, last_valid_index,
                               articulation_index, articulation_yaw, articulation_has_forward))
      {
        goal_index = articulation_index;
        goal_yaw = articulation_yaw;
        has_forward_direction = articulation_has_forward;
      }
    }
  }

  ROS_ASSERT(goal_index <= last_valid_index);

  worldToGridBounded(info, plan[goal_index].x, plan[goal_index].y, x, y);
  desired_angle = goal_yaw;

  bool same_as_held = false;
  if (holding_goal_)
  {
    double position_diff_x = plan[goal_index].x - held_goal_pose_.x;
    double position_diff_y = plan[goal_index].y - held_goal_pose_.y;
    double yaw_diff = angles::shortest_angular_distance(goal_yaw, held_goal_pose_.theta);
    same_as_held = (fabs(position_diff_x) <= hold_position_epsilon_) &&
                   (fabs(position_diff_y) <= hold_position_epsilon_) &&
                   (fabs(yaw_diff) <= hold_yaw_epsilon_);
  }

  if (!same_as_held)
  {
    held_goal_pose_ = plan[goal_index];
    held_goal_pose_.theta = goal_yaw;
    held_goal_index_ = goal_index;
  }
  else
  {
    held_goal_pose_.x = plan[goal_index].x;
    held_goal_pose_.y = plan[goal_index].y;
    held_goal_pose_.theta = goal_yaw;
  }
  holding_goal_ = true;

  ROS_DEBUG_NAMED("PathProgressCritic",
                  "Selected goal index %u (x: %.3f, y: %.3f, yaw: %.3f rad). last_progress_index_: %u",
                  goal_index, plan[goal_index].x, plan[goal_index].y, goal_yaw, last_progress_index_);

  publishIntermediateGoal(held_goal_pose_, held_goal_pose_.theta);
  return true;
}

unsigned int PathProgressCritic::getGoalIndex(const std::vector<geometry_msgs::Pose2D>& plan, unsigned int start_index,
                                              unsigned int last_valid_index, double& desired_angle,
                                              bool& has_forward_direction) const
{
  if (plan.empty())
  {
    desired_angle = 0.0;
    has_forward_direction = false;
    return 0;
  }

  const double epsilon = 1e-9;
  unsigned int clamped_start = std::min(start_index, static_cast<unsigned int>(plan.size() - 1));
  unsigned int clamped_last = std::min(last_valid_index, static_cast<unsigned int>(plan.size() - 1));

  if (clamped_start >= clamped_last)
  {
    has_forward_direction = computeOutgoingAngle(plan, clamped_start, desired_angle);
    if (!has_forward_direction)
    {
      desired_angle = plan[clamped_start].theta;
    }
    return clamped_start;
  }

  unsigned int goal_index = clamped_start;
  double base_angle = 0.0;
  bool base_angle_set = false;
  double previous_segment_angle = 0.0;
  bool previous_segment_angle_set = false;
  unsigned int previous_segment_end_index = clamped_start;
  double last_valid_segment_angle = 0.0;
  bool last_valid_segment_angle_set = false;

  for (unsigned int i = clamped_start + 1; i <= clamped_last; ++i)
  {
    double direction_x = plan[i].x - plan[i - 1].x;
    double direction_y = plan[i].y - plan[i - 1].y;
    double length = hypot(direction_x, direction_y);
    if (length < epsilon)
    {
      continue;
    }

    double current_angle = atan2(direction_y, direction_x);
    last_valid_segment_angle = current_angle;
    last_valid_segment_angle_set = true;

    if (!base_angle_set)
    {
      base_angle = current_angle;
      base_angle_set = true;
    }

    if (previous_segment_angle_set)
    {
      double articulation_angle = fabs(angles::shortest_angular_distance(previous_segment_angle, current_angle));
      if (articulation_angle >= articulation_angle_threshold_)
      {
        goal_index = previous_segment_end_index;
        break;
      }
    }

    double deviation = fabs(angles::shortest_angular_distance(base_angle, current_angle));
    if (deviation > angle_threshold_)
    {
      break;
    }

    goal_index = i;
    previous_segment_angle = current_angle;
    previous_segment_end_index = i;
    previous_segment_angle_set = true;
  }

  has_forward_direction = computeOutgoingAngle(plan, goal_index, desired_angle);
  if (!has_forward_direction)
  {
    if (goal_index == plan.size() - 1)
    {
      desired_angle = plan[goal_index].theta;
    }
    else if (previous_segment_angle_set)
    {
      desired_angle = previous_segment_angle;
      has_forward_direction = true;
    }
    else if (last_valid_segment_angle_set)
    {
      desired_angle = last_valid_segment_angle;
      has_forward_direction = true;
    }
    else
    {
      desired_angle = plan[goal_index].theta;
    }
  }

  if (goal_index == plan.size() - 1)
  {
    has_forward_direction = false;
  }

  return goal_index;
}

bool PathProgressCritic::findNextArticulation(const std::vector<geometry_msgs::Pose2D>& plan, unsigned int start_index,
                                              unsigned int end_index, unsigned int last_valid_index,
                                              unsigned int& articulation_index, double& articulation_yaw,
                                              bool& has_forward_direction) const
{
  const double epsilon = 1e-9;
  articulation_index = 0;
  articulation_yaw = 0.0;
  has_forward_direction = false;

  if (plan.size() < 2)
  {
    return false;
  }

  unsigned int clamped_end = std::min(end_index, static_cast<unsigned int>(plan.size() - 1));
  clamped_end = std::min(clamped_end, last_valid_index);
  if (clamped_end < 1)
  {
    return false;
  }

  unsigned int scan_start = std::max(start_index, 1u);
  if (scan_start > clamped_end)
  {
    return false;
  }

  double previous_segment_angle = 0.0;
  bool previous_segment_angle_set = false;
  unsigned int previous_segment_end_index = 0;

  for (unsigned int i = scan_start; i <= clamped_end; ++i)
  {
    double direction_x = plan[i].x - plan[i - 1].x;
    double direction_y = plan[i].y - plan[i - 1].y;
    double length = hypot(direction_x, direction_y);
    if (length < epsilon)
    {
      continue;
    }

    double current_angle = atan2(direction_y, direction_x);

    if (previous_segment_angle_set)
    {
      double articulation_angle = fabs(angles::shortest_angular_distance(previous_segment_angle, current_angle));
      if (articulation_angle >= articulation_angle_threshold_)
      {
        articulation_index = previous_segment_end_index;
        articulation_yaw = current_angle;
        has_forward_direction = true;
        if (!computeOutgoingAngle(plan, articulation_index, articulation_yaw))
        {
          has_forward_direction = false;
          if (articulation_index == plan.size() - 1)
          {
            articulation_yaw = plan[articulation_index].theta;
          }
        }
        return true;
      }
    }

    previous_segment_angle = current_angle;
    previous_segment_angle_set = true;
    previous_segment_end_index = i;
  }

  return false;
}

bool PathProgressCritic::computeOutgoingAngle(const std::vector<geometry_msgs::Pose2D>& plan, unsigned int index,
                                              double& angle) const
{
  const double epsilon = 1e-9;
  if (plan.empty() || index >= plan.size())
  {
    return false;
  }

  for (unsigned int i = index + 1; i < plan.size(); ++i)
  {
    double dx = plan[i].x - plan[index].x;
    double dy = plan[i].y - plan[index].y;
    double length = hypot(dx, dy);
    if (length >= epsilon)
    {
      angle = atan2(dy, dx);
      return true;
    }
  }

  return false;
}

bool PathProgressCritic::isGoalReached(const geometry_msgs::Pose2D& robot_pose, const geometry_msgs::Pose2D& goal_pose,
                                       double goal_yaw) const
{
  double distance = nav_2d_utils::poseDistance(goal_pose, robot_pose);
  double yaw_error = fabs(angles::shortest_angular_distance(robot_pose.theta, goal_yaw));

  return distance < xy_local_goal_tolerance_ && yaw_error < yaw_local_goal_tolerance_;
}

}  // namespace mir_dwb_critics

PLUGINLIB_EXPORT_CLASS(mir_dwb_critics::PathProgressCritic, dwb_local_planner::TrajectoryCritic)
