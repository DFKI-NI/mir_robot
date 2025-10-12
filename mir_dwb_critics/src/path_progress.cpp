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
#include <nav_grid/coordinate_conversion.h>
#include <pluginlib/class_list_macros.h>
#include <nav_2d_utils/path_ops.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
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
  if (!private_nh.getParam("xy_goal_tolerance", final_goal_xy_tolerance_))
  {
    critic_nh_.param("xy_goal_tolerance", final_goal_xy_tolerance_, xy_local_goal_tolerance_);
  }
  critic_nh_.param("angle_threshold", angle_threshold_, M_PI_4);
  critic_nh_.param("articulation_angle_threshold", articulation_angle_threshold_, 1.3089969389957472);
  critic_nh_.param("heading_scale", heading_scale_, 1.0);
  critic_nh_.param("enforce_forward_dot", enforce_forward_dot_, true);
  critic_nh_.param("always_target_articulations", always_target_articulations_, true);
  initial_alignment_done_ = false;

  intermediate_goal_pub_ = critic_nh_.advertise<geometry_msgs::PoseStamped>("intermediate_goal", 1);
  articulation_points_pub_ = critic_nh_.advertise<sensor_msgs::PointCloud>("articulation_points", 1);
  intermediate_goal_tolerance_pub_ =
      critic_nh_.advertise<visualization_msgs::MarkerArray>("intermediate_goal_tolerance", 1);

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
  final_goal_xy_tolerance_ = std::max(final_goal_xy_tolerance_, 0.0);
  have_last_plan_ = false;
  last_plan_size_ = 0;
  last_plan_end_pose_.x = 0.0;
  last_plan_end_pose_.y = 0.0;
  last_plan_end_pose_.theta = 0.0;
  last_plan_frame_id_.clear();
  last_plan_stamp_ = ros::Time(0);
  last_plan_.clear();
  plan_position_epsilon_ = 1e-4;
  plan_yaw_epsilon_ = 1e-4;
  critic_nh_.param("plan_alignment_position_tolerance", plan_alignment_position_tolerance_, 0.15);
  critic_nh_.param("plan_alignment_yaw_tolerance", plan_alignment_yaw_tolerance_, 3.14159265358979323846);
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
  initial_alignment_done_ = false;
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

  auto plansEqual = [&](const std::vector<geometry_msgs::Pose2D>& a,
                        const std::vector<geometry_msgs::Pose2D>& b) {
    if (a.size() != b.size())
    {
      return false;
    }

    for (size_t idx = 0; idx < a.size(); ++idx)
    {
      if (fabs(a[idx].x - b[idx].x) > plan_position_epsilon_ ||
          fabs(a[idx].y - b[idx].y) > plan_position_epsilon_ ||
          fabs(angles::shortest_angular_distance(a[idx].theta, b[idx].theta)) > plan_yaw_epsilon_)
      {
        return false;
      }
    }

    return true;
  };

  auto matchPoseToPlanIndex = [&](const geometry_msgs::Pose2D& pose, unsigned int& index_out) {
    bool found = false;
    double best_distance = std::numeric_limits<double>::infinity();

    for (unsigned int idx = 0; idx < plan.size(); ++idx)
    {
      double dx = plan[idx].x - pose.x;
      double dy = plan[idx].y - pose.y;
      double distance = hypot(dx, dy);
      if (distance > plan_alignment_position_tolerance_)
      {
        continue;
      }

      double yaw_error = fabs(angles::shortest_angular_distance(plan[idx].theta, pose.theta));
      if (yaw_error > plan_alignment_yaw_tolerance_)
      {
        continue;
      }

      if (distance < best_distance)
      {
        best_distance = distance;
        index_out = idx;
        found = true;
      }
    }

    return found;
  };

  // Reset state if a new global plan arrives (size, geometry, or frame changes)
  bool plan_changed = false;
  if (!have_last_plan_)
  {
    plan_changed = true;
  }
  else
  {
    if (global_plan.header.frame_id != last_plan_frame_id_)
    {
      plan_changed = true;
    }
    else if (!plansEqual(plan, last_plan_))
    {
      plan_changed = true;
    }
  }

  if (plan_changed)
  {
    bool state_preserved = false;
    bool progress_match_found = false;
    unsigned int restored_progress_index = 0u;
    std::vector<std::pair<unsigned int, geometry_msgs::Pose2D>> preserved_reached;
    preserved_reached.reserve(reached_intermediate_goals_.size());

    unsigned int previous_progress_index = last_progress_index_;

    for (const auto& reached_pose : reached_intermediate_goals_)
    {
      unsigned int matched_index = 0u;
      if (matchPoseToPlanIndex(reached_pose, matched_index))
      {
        geometry_msgs::Pose2D preserved_pose = reached_pose;
        preserved_pose.x = plan[matched_index].x;
        preserved_pose.y = plan[matched_index].y;
        preserved_reached.emplace_back(matched_index, preserved_pose);
        state_preserved = true;
        progress_match_found = true;
        restored_progress_index = std::max(restored_progress_index, matched_index);
      }
    }

    unsigned int held_match_index = 0u;
    if (holding_goal_)
    {
      if (matchPoseToPlanIndex(held_goal_pose_, held_match_index))
      {
        state_preserved = true;
        held_goal_index_ = held_match_index;
        held_goal_pose_.x = plan[held_match_index].x;
        held_goal_pose_.y = plan[held_match_index].y;
      }
      else
      {
        holding_goal_ = false;
        held_goal_index_ = 0u;
        held_goal_pose_.x = 0.0;
        held_goal_pose_.y = 0.0;
        held_goal_pose_.theta = 0.0;
      }
    }

    unsigned int robot_match_index = 0u;
    if (matchPoseToPlanIndex(robot_pose, robot_match_index))
    {
      state_preserved = true;

      double match_goal_yaw = plan[robot_match_index].theta;
      if (!computeOutgoingAngle(plan, robot_match_index, match_goal_yaw))
      {
        match_goal_yaw = plan[robot_match_index].theta;
      }

      bool match_is_final_index = (robot_match_index + 1u) >= plan.size();
      double match_yaw_tolerance = match_is_final_index ? final_goal_yaw_tolerance_ : yaw_local_goal_tolerance_;
      double yaw_error = fabs(angles::shortest_angular_distance(robot_pose.theta, match_goal_yaw));

      if (yaw_error <= match_yaw_tolerance)
      {
        progress_match_found = true;
        restored_progress_index = std::max(restored_progress_index, robot_match_index);
      }
      else
      {
        ROS_DEBUG_NAMED("PathProgressCritic",
                        "Robot pose matched plan index %u but yaw error %.3f rad exceeds tolerance %.3f."
                        " Preserving state without increasing progress.",
                        robot_match_index, yaw_error, match_yaw_tolerance);
      }
    }

    if (state_preserved)
    {
      if (progress_match_found)
      {
        std::sort(preserved_reached.begin(), preserved_reached.end(),
                  [](const std::pair<unsigned int, geometry_msgs::Pose2D>& lhs,
                     const std::pair<unsigned int, geometry_msgs::Pose2D>& rhs) {
                    return lhs.first < rhs.first;
                  });
        reached_intermediate_goals_.clear();
        reached_intermediate_goals_.reserve(preserved_reached.size());
        for (const auto& entry : preserved_reached)
        {
          reached_intermediate_goals_.push_back(entry.second);
        }
        unsigned int plan_last_index = plan.empty() ? 0u : static_cast<unsigned int>(plan.size() - 1);
        last_progress_index_ = std::min(restored_progress_index, plan_last_index);
      }
      else
      {
        reached_intermediate_goals_.clear();
        unsigned int plan_last_index = plan.empty() ? 0u : static_cast<unsigned int>(plan.size() - 1);
        last_progress_index_ = std::min(previous_progress_index, plan_last_index);
      }
    }
    else
    {
      reached_intermediate_goals_.clear();
      last_progress_index_ = 0u;
      holding_goal_ = false;
      held_goal_index_ = 0u;
      held_goal_pose_.x = 0.0;
      held_goal_pose_.y = 0.0;
      held_goal_pose_.theta = 0.0;
      initial_alignment_done_ = false;
    }

    have_last_plan_ = true;
  }

  last_plan_size_ = plan.size();
  last_plan_end_pose_ = plan.back();
  last_plan_frame_id_ = global_plan.header.frame_id;
  last_plan_stamp_ = global_plan.header.stamp;
  last_plan_ = plan;


  if (plan.empty())
  {
    ROS_ERROR_NAMED("PathProgressCritic", "The adjusted global plan was empty.");
    return false;
  }

  unsigned int plan_last_index = static_cast<unsigned int>(plan.size() - 1);

  if (initial_alignment_done_ && plan.size() > 1 && last_progress_index_ > 0)
  {
    unsigned int prune_limit = std::min(last_progress_index_, plan_last_index);
    if (prune_limit > 0)
    {
      plan.erase(plan.begin(), plan.begin() + prune_limit);
      last_progress_index_ = last_progress_index_ >= prune_limit ? last_progress_index_ - prune_limit : 0;
      plan_last_index = static_cast<unsigned int>(plan.size() - 1);

      if (holding_goal_)
      {
        if (held_goal_index_ < prune_limit)
        {
          holding_goal_ = false;
          held_goal_index_ = 0;
        }
        else
        {
          held_goal_index_ -= prune_limit;
        }
      }
    }
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

  auto collectArticulationIndices = [&](unsigned int scan_start, unsigned int scan_end) {
    std::vector<unsigned int> articulation_indices;
    if (plan.size() < 2)
    {
      return articulation_indices;
    }

    const double epsilon = 1e-9;
    unsigned int clamped_start = std::max(scan_start, 1u);
    unsigned int clamped_end = std::min(scan_end, plan_last_index);
    if (clamped_start > clamped_end)
    {
      return articulation_indices;
    }

    double previous_segment_angle = 0.0;
    bool previous_segment_angle_set = false;
    unsigned int previous_segment_end_index = 0u;

    for (unsigned int i = clamped_start; i <= clamped_end; ++i)
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
        double articulation_angle =
            fabs(angles::shortest_angular_distance(previous_segment_angle, current_angle));
        if (articulation_angle >= articulation_angle_threshold_)
        {
          if (articulation_indices.empty() || articulation_indices.back() != previous_segment_end_index)
          {
            articulation_indices.push_back(previous_segment_end_index);
          }
        }
      }

      previous_segment_angle = current_angle;
      previous_segment_angle_set = true;
      previous_segment_end_index = i;
    }

    return articulation_indices;
  };

  auto publishArticulationPointCloud = [&](const std::vector<unsigned int>& articulation_indices) {
    if (!articulation_points_pub_)
    {
      return;
    }

    sensor_msgs::PointCloud cloud_msg;
    cloud_msg.header = global_plan.header;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.points.reserve(articulation_indices.size());

    for (unsigned int index : articulation_indices)
    {
      if (index >= plan.size())
      {
        continue;
      }

      geometry_msgs::Point32 point;
      point.x = plan[index].x;
      point.y = plan[index].y;
      point.z = 0.0;
      cloud_msg.points.push_back(point);
    }

    articulation_points_pub_.publish(cloud_msg);
  };

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

    if (intermediate_goal_tolerance_pub_)
    {
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker marker;
      marker.header = goal_msg.header;
      marker.header.stamp = goal_msg.header.stamp;
      marker.ns = "intermediate_goal_tolerance";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = goal_msg.pose.position;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      double diameter = std::max(2.0 * xy_local_goal_tolerance_, 1e-6);
      marker.scale.x = diameter;
      marker.scale.y = diameter;
      marker.scale.z = diameter;
      marker.color.r = 0.2f;
      marker.color.g = 0.8f;
      marker.color.b = 0.4f;
      marker.color.a = 0.35f;
      marker.lifetime = ros::Duration(0.0);
      marker_array.markers.push_back(marker);
      intermediate_goal_tolerance_pub_.publish(marker_array);
    }
  };

  if (!initial_alignment_done_)
  {
    double desired_initial_yaw = plan.front().theta;
    if (!plan.empty())
    {
      double outgoing_angle = desired_initial_yaw;
      if (computeOutgoingAngle(plan, 0u, outgoing_angle))
      {
        desired_initial_yaw = outgoing_angle;
      }
    }

    double yaw_error = fabs(angles::shortest_angular_distance(robot_pose.theta, desired_initial_yaw));
    if (yaw_error >= yaw_local_goal_tolerance_)
    {
      unsigned int rx = 0;
      unsigned int ry = 0;
      if (worldToGridBounded(info, robot_pose.x, robot_pose.y, rx, ry))
      {
        x = rx;
        y = ry;
        desired_angle = desired_initial_yaw;
        held_goal_pose_.x = robot_pose.x;
        held_goal_pose_.y = robot_pose.y;
        held_goal_pose_.theta = desired_initial_yaw;
        held_goal_index_ = 0;
        holding_goal_ = true;
        publishIntermediateGoal(held_goal_pose_, held_goal_pose_.theta);
        ROS_DEBUG_NAMED("PathProgressCritic",
                        "Initial alignment: rotate in place to yaw %.3f rad at robot pose (x: %.3f, y: %.3f)",
                        held_goal_pose_.theta, held_goal_pose_.x, held_goal_pose_.y);
        return true;
      }
    }

    initial_alignment_done_ = true;
  }

  std::vector<unsigned int> articulation_indices = collectArticulationIndices(1u, plan_last_index);
  publishArticulationPointCloud(articulation_indices);

  if (holding_goal_)
  {
    unsigned int held_x = 0;
    unsigned int held_y = 0;
    bool held_in_costmap = worldToGridBounded(info, held_goal_pose_.x, held_goal_pose_.y, held_x, held_y);

    if (!held_in_costmap)
    {
      ROS_WARN_NAMED("PathProgressCritic",
                     "Held goal (index %u) is outside the local costmap. Releasing hold to search for a new goal.",
                     held_goal_index_);
      holding_goal_ = false;
      held_goal_index_ = 0;
    }
    else
    {
      double held_xy_tolerance = (held_goal_index_ == plan_last_index) ? final_goal_xy_tolerance_ : xy_local_goal_tolerance_;
      double held_yaw_tolerance = (held_goal_index_ == plan_last_index) ? final_goal_yaw_tolerance_ : yaw_local_goal_tolerance_;

      double yaw_error = fabs(angles::shortest_angular_distance(robot_pose.theta, held_goal_pose_.theta));
      if (yaw_error >= held_yaw_tolerance)
      {
        x = held_x;
        y = held_y;
        desired_angle = held_goal_pose_.theta;
        publishIntermediateGoal(held_goal_pose_, held_goal_pose_.theta);
        ROS_DEBUG_NAMED("PathProgressCritic", "Holding goal index %u due to yaw error %.3f rad (threshold %.3f)",
                        held_goal_index_, yaw_error, held_yaw_tolerance);
        return true;
      }

      if (!isPoseReached(robot_pose, held_goal_pose_, held_goal_pose_.theta, held_xy_tolerance, held_yaw_tolerance))
      {
        x = held_x;
        y = held_y;
        desired_angle = held_goal_pose_.theta;
        publishIntermediateGoal(held_goal_pose_, held_goal_pose_.theta);
        ROS_DEBUG_NAMED("PathProgressCritic",
                        "Holding goal index %u until full pose tolerance satisfied (XY + yaw)", held_goal_index_);
        return true;
      }

      last_progress_index_ = std::max(last_progress_index_, held_goal_index_);
      geometry_msgs::Pose2D reached_pose = held_goal_pose_;
      if (reached_intermediate_goals_.empty() ||
          nav_2d_utils::poseDistance(reached_intermediate_goals_.back(), reached_pose) > 1e-6 ||
          fabs(angles::shortest_angular_distance(reached_intermediate_goals_.back().theta, reached_pose.theta)) > 1e-6)
      {
        reached_intermediate_goals_.push_back(reached_pose);
      }
      ROS_DEBUG_NAMED("PathProgressCritic",
                      "Reached held intermediate goal index %u while respecting pose tolerances. last_progress_index_: %u",
                      held_goal_index_, last_progress_index_);
      holding_goal_ = false;
    }
  }

  // Constrain the search range to enforce monotonic progress with the updated bookkeeping.
  last_progress_index_ = std::min(last_progress_index_, plan_last_index);

  unsigned int search_start_index = std::max(start_index, last_progress_index_);
  search_start_index = std::min(search_start_index, last_valid_index);

  if (holding_goal_ && held_goal_index_ >= last_progress_index_)
  {
    search_start_index = std::min(search_start_index, held_goal_index_);
  }

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

  auto articulationSearchStart = [&](unsigned int candidate_start) {
    return std::max({candidate_start, articulation_scan_start, 1u});
  };

  unsigned int goal_index = search_start_index;
  double goal_yaw = plan[goal_index].theta;
  bool has_forward_direction = false;
  bool found_goal = false;
  bool forced_skipped_articulation = false;
  unsigned int next_articulation_index = 0;
  double next_articulation_yaw = 0.0;
  bool next_articulation_has_forward = false;
  bool has_next_articulation = false;

  if (initial_alignment_done_ && !articulation_indices.empty())
  {
    unsigned int articulation_window_start = std::max(search_start_index, last_progress_index_ + 1);
    unsigned int articulation_window_end = last_valid_index;

    for (unsigned int idx : articulation_indices)
    {
      if (idx < articulation_window_start || idx > articulation_window_end)
      {
        continue;
      }

      double articulation_yaw = plan[idx].theta;
      bool articulation_has_forward = computeOutgoingAngle(plan, idx, articulation_yaw);
      double articulation_goal_yaw = articulation_has_forward ? articulation_yaw : plan[idx].theta;

      double articulation_xy_tolerance = (idx == plan_last_index) ? final_goal_xy_tolerance_ : xy_local_goal_tolerance_;
      double articulation_yaw_tolerance = (idx == plan_last_index) ? final_goal_yaw_tolerance_ : yaw_local_goal_tolerance_;

      if (isPoseReached(robot_pose, plan[idx], articulation_goal_yaw, articulation_xy_tolerance,
                        articulation_yaw_tolerance))
      {
        last_progress_index_ = std::max(last_progress_index_, idx);
        geometry_msgs::Pose2D reached_pose = plan[idx];
        reached_pose.theta = articulation_goal_yaw;
        if (reached_intermediate_goals_.empty() ||
            nav_2d_utils::poseDistance(reached_intermediate_goals_.back(), reached_pose) > 1e-6 ||
            fabs(angles::shortest_angular_distance(reached_intermediate_goals_.back().theta, reached_pose.theta)) >
                1e-6)
        {
          reached_intermediate_goals_.push_back(reached_pose);
        }
        continue;
      }

      next_articulation_index = idx;
      next_articulation_yaw = articulation_goal_yaw;
      next_articulation_has_forward = articulation_has_forward;
      has_next_articulation = true;
      break;
    }
  }

  if (!found_goal && initial_alignment_done_ && search_start_index > last_progress_index_ + 1 &&
      !articulation_indices.empty())
  {
    unsigned int articulation_lower_bound = std::max(last_progress_index_ + 1, 1u);
    unsigned int articulation_upper_bound = std::min(search_start_index - 1, last_valid_index);

    if (articulation_lower_bound <= articulation_upper_bound)
    {
      auto articulation_it = std::find_if(articulation_indices.begin(), articulation_indices.end(),
                                          [&](unsigned int index) {
                                            return index >= articulation_lower_bound && index <= articulation_upper_bound;
                                          });

      if (articulation_it != articulation_indices.end())
      {
        goal_index = *articulation_it;
        has_forward_direction = computeOutgoingAngle(plan, goal_index, goal_yaw);
        if (!has_forward_direction)
        {
          goal_yaw = plan[goal_index].theta;
        }

        if (enforce_forward_dot_ && has_forward_direction)
        {
          double to_goal_x = plan[goal_index].x - robot_pose.x;
          double to_goal_y = plan[goal_index].y - robot_pose.y;
          double dot = to_goal_x * std::cos(goal_yaw) + to_goal_y * std::sin(goal_yaw);
          if (dot < 0.0)
          {
            ROS_WARN_NAMED("PathProgressCritic",
                           "Forcing skipped articulation index %u despite backward dot product %.3f due to policy.",
                           goal_index, dot);
          }
        }

        forced_skipped_articulation = true;
        found_goal = true;
        ROS_DEBUG_NAMED("PathProgressCritic",
                        "Recovered skipped articulation index %u between progress %u and search start %u.",
                        goal_index, last_progress_index_, search_start_index);
      }
    }
  }

  unsigned int search_index = search_start_index;
  while (!forced_skipped_articulation && search_index <= last_valid_index)
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
      unsigned int articulation_start_index = articulationSearchStart(search_index);
      if (articulation_start_index <= candidate_index &&
          findNextArticulation(plan, articulation_start_index, candidate_index, last_valid_index,
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

    if (has_next_articulation && candidate_index >= next_articulation_index)
    {
      goal_index = next_articulation_index;
      goal_yaw = next_articulation_yaw;
      has_forward_direction = next_articulation_has_forward;
      found_goal = true;
      forced_skipped_articulation = true;
      has_next_articulation = false;
      ROS_DEBUG_NAMED("PathProgressCritic",
                      "Selecting pending articulation index %u reached at candidate %u.", goal_index,
                      candidate_index);
      break;
    }

    double candidate_xy_tolerance = (candidate_index == plan_last_index) ? final_goal_xy_tolerance_ : xy_local_goal_tolerance_;
    double candidate_yaw_tolerance = (candidate_index == plan_last_index) ? final_goal_yaw_tolerance_ : yaw_local_goal_tolerance_;

    if (isPoseReached(robot_pose, plan[candidate_index], candidate_yaw, candidate_xy_tolerance,
                      candidate_yaw_tolerance))
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

    if (has_next_articulation && goal_index >= next_articulation_index)
    {
      goal_index = next_articulation_index;
      goal_yaw = next_articulation_yaw;
      has_forward_direction = next_articulation_has_forward;
      has_next_articulation = false;
      forced_skipped_articulation = true;
      ROS_DEBUG_NAMED("PathProgressCritic",
                      "Fallback switching to pending articulation index %u.", goal_index);
    }

    if (goal_index > last_progress_index_)
    {
      unsigned int articulation_index = 0;
      double articulation_yaw = goal_yaw;
      bool articulation_has_forward = has_forward_direction;
      unsigned int articulation_start_index = articulationSearchStart(search_start_index);
      if (articulation_start_index <= goal_index &&
          findNextArticulation(plan, articulation_start_index, goal_index, last_valid_index,
                               articulation_index, articulation_yaw, articulation_has_forward))
      {
        goal_index = articulation_index;
        goal_yaw = articulation_yaw;
        has_forward_direction = articulation_has_forward;
      }
    }
  }

  if (plan_last_index <= last_valid_index && goal_index != plan_last_index)
  {
    double snap_threshold = final_goal_xy_tolerance_;
    if (snap_threshold >= 0.0)
    {
      double final_dx = plan[plan_last_index].x - plan[goal_index].x;
      double final_dy = plan[plan_last_index].y - plan[goal_index].y;
      double distance_to_final = hypot(final_dx, final_dy);
      if (distance_to_final <= snap_threshold)
      {
        ROS_DEBUG_NAMED("PathProgressCritic",
                        "Snapping intermediate goal index %u to final goal because distance %.3f <= threshold %.3f",
                        goal_index, distance_to_final, snap_threshold);
        goal_index = plan_last_index;
        goal_yaw = plan[plan_last_index].theta;
        has_forward_direction = false;
        forced_skipped_articulation = false;
        has_next_articulation = false;
      }
    }
  }

  bool pending_articulation = false;
  if (last_progress_index_ < goal_index)
  {
    pending_articulation = std::find(articulation_indices.begin(), articulation_indices.end(), goal_index) !=
                           articulation_indices.end();
  }

  // Only consider snapping to the final goal yaw when we have already selected the last path index as goal.
  // Otherwise, keep following the path even if the final goal is within tolerance.
  if (!pending_articulation && final_goal_xy_tolerance_ >= 0.0 && plan_last_index <= last_valid_index &&
      goal_index == plan_last_index)
  {
    double final_dx = plan[plan_last_index].x - plan[goal_index].x;
    double final_dy = plan[plan_last_index].y - plan[goal_index].y;
    double final_distance = hypot(final_dx, final_dy);
    if (final_distance <= final_goal_xy_tolerance_)
    {
      goal_index = plan_last_index;
      goal_yaw = plan[plan_last_index].theta;
      has_forward_direction = false;
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

bool PathProgressCritic::isPoseReached(const geometry_msgs::Pose2D& robot_pose, const geometry_msgs::Pose2D& goal_pose,
                                       double goal_yaw, double xy_tolerance, double yaw_tolerance) const
{
  double distance = nav_2d_utils::poseDistance(goal_pose, robot_pose);
  double yaw_error = fabs(angles::shortest_angular_distance(robot_pose.theta, goal_yaw));

  return distance < xy_tolerance && yaw_error < yaw_tolerance;
}

bool PathProgressCritic::isGoalReached(const geometry_msgs::Pose2D& robot_pose, const geometry_msgs::Pose2D& goal_pose,
                                       double goal_yaw) const
{
  return isPoseReached(robot_pose, goal_pose, goal_yaw, xy_local_goal_tolerance_, yaw_local_goal_tolerance_);
}

}  // namespace mir_dwb_critics

PLUGINLIB_EXPORT_CLASS(mir_dwb_critics::PathProgressCritic, dwb_local_planner::TrajectoryCritic)
