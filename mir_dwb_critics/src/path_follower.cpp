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
#include <mir_dwb_critics/path_follower.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core2/exceptions.h>
#include <nav_grid/coordinate_conversion.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace mir_dwb_critics
{
namespace
{
constexpr double kInfinity = std::numeric_limits<double>::infinity();

geometry_msgs::Pose2D makePose(double x, double y, double theta)
{
  geometry_msgs::Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}
}  // namespace

void PathFollowerCritic::onInit()
{
  critic_nh_.param("xy_local_goal_tolerance", xy_local_goal_tolerance_, 0.05);
  critic_nh_.param("xy_final_goal_tolerance", xy_final_goal_tolerance_, 0.02);
  critic_nh_.param("yaw_local_goal_tolerance", yaw_local_goal_tolerance_, 0.1);
  critic_nh_.param("stop_turn_angle", stop_turn_angle_, 0.5);
  critic_nh_.param("start_turn_angle", start_turn_angle_, 0.2);

  // Velocity profile limits default to the planner's kinematic configuration.
  double max_vel_x = 0.8, max_vel_theta = 1.0, acc_lim_x = 1.5, acc_lim_theta = 2.0, decel_lim_x, decel_lim_theta;
  planner_nh_.param("max_vel_x", max_vel_x, max_vel_x);
  planner_nh_.param("max_vel_theta", max_vel_theta, max_vel_theta);
  planner_nh_.param("acc_lim_x", acc_lim_x, acc_lim_x);
  planner_nh_.param("acc_lim_theta", acc_lim_theta, acc_lim_theta);
  planner_nh_.param("decel_lim_x", decel_lim_x, -acc_lim_x);
  planner_nh_.param("decel_lim_theta", decel_lim_theta, -acc_lim_theta);
  critic_nh_.param("max_speed", max_speed_, std::fabs(max_vel_x));
  critic_nh_.param("max_rotation", max_rotation_, std::fabs(max_vel_theta));
  // Braking at half the kinematic limit leaves margin for the proportional final approach.
  critic_nh_.param("decel", decel_, 0.5 * std::fabs(decel_lim_x));
  critic_nh_.param("rot_decel", rot_decel_, 0.5 * std::fabs(decel_lim_theta));
  critic_nh_.param("approach_gain", approach_gain_, 3.0);
  critic_nh_.param("rotation_gain", rotation_gain_, 4.0);
  max_speed_ = std::max(max_speed_, 1e-3);
  max_rotation_ = std::max(max_rotation_, 1e-3);

  critic_nh_.param("speed_scale", speed_scale_, 64.0);
  critic_nh_.param("rotation_scale", rotation_scale_, 64.0);
  critic_nh_.param("heading_scale", heading_scale_, 25.0);
  critic_nh_.param("path_distance_scale", path_distance_scale_, 4.0);
  critic_nh_.param("search_window", search_window_, 1.0);
  critic_nh_.param("corner_fillet_length", corner_fillet_length_, 0.3);
  critic_nh_.param("curve_rotation", curve_rotation_, 0.9 * max_rotation_);
  critic_nh_.param("curve_lookahead", curve_lookahead_, 1.0);
  critic_nh_.param("plan_alignment_position_tolerance", plan_alignment_position_tolerance_, 0.15);

  intermediate_goal_pub_ = critic_nh_.advertise<geometry_msgs::PoseStamped>("intermediate_goal", 1);
  reset();
}

void PathFollowerCritic::reset()
{
  prepared_ = false;
  turning_ = false;
  vertices_.clear();
  last_plan_.clear();
  have_progress_ = false;
  have_passed_stop_ = false;
  have_turning_stop_ = false;
}

double PathFollowerCritic::desiredSpeed(double distance_to_stop) const
{
  if (!(distance_to_stop > 0.0))
  {
    return 0.0;
  }
  double speed = std::min(max_speed_, approach_gain_ * distance_to_stop);
  return std::min(speed, std::sqrt(2.0 * decel_ * distance_to_stop));
}

double PathFollowerCritic::desiredRotation(double heading_error) const
{
  double magnitude = std::min(max_rotation_, rotation_gain_ * std::fabs(heading_error));
  magnitude = std::min(magnitude, std::sqrt(2.0 * rot_decel_ * std::fabs(heading_error)));
  return heading_error < 0.0 ? -magnitude : magnitude;
}

std::vector<PathFollowerCritic::Vertex> PathFollowerCritic::buildPolyline(const nav_2d_msgs::Path2D& plan,
                                                                          const geometry_msgs::Pose2D& goal) const
{
  std::vector<Vertex> vertices;
  const double merge_distance = 1e-3;
  for (const auto& pose : plan.poses)
  {
    // In-place turns are encoded as several poses with identical XY; keep the first and last heading.
    if (!vertices.empty() && std::hypot(pose.x - vertices.back().x, pose.y - vertices.back().y) < merge_distance)
    {
      vertices.back().theta_out = pose.theta;
      continue;
    }
    Vertex vertex;
    vertex.x = pose.x;
    vertex.y = pose.y;
    vertex.theta_in = pose.theta;
    vertex.theta_out = pose.theta;
    vertices.push_back(vertex);
  }
  if (vertices.empty())
  {
    return vertices;
  }

  for (size_t i = 0; i + 1 < vertices.size(); ++i)
  {
    Vertex& from = vertices[i];
    Vertex& to = vertices[i + 1];
    double dx = to.x - from.x, dy = to.y - from.y;
    to.s = from.s + std::hypot(dx, dy);
    from.forward = dx * std::cos(from.theta_out) + dy * std::sin(from.theta_out) >= 0.0;
  }
  if (vertices.size() > 1)
  {
    vertices.back().forward = vertices[vertices.size() - 2].forward;
  }

  // The transformed plan may be cropped to the local costmap; only a plan ending at the goal ends with a stop.
  const Vertex& last = vertices.back();
  bool ends_at_goal = std::hypot(last.x - goal.x, last.y - goal.y) <= plan_alignment_position_tolerance_;
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    Vertex& vertex = vertices[i];
    double turn = std::fabs(angles::shortest_angular_distance(vertex.theta_in, vertex.theta_out));
    bool reversal = i > 0 && i + 1 < vertices.size() && vertices[i - 1].forward != vertex.forward;
    bool is_final = i + 1 == vertices.size() && ends_at_goal;
    // Driving through a small turn only pays off while moving; at the plan start the robot stands still, and
    // accelerating into a 22.5 degree kink swerved it 10 cm off the path.
    bool initial = i == 0 && turn >= start_turn_angle_;
    vertex.stop = turn >= stop_turn_angle_ || reversal || is_final || initial;
  }
  // A smaller initial mismatch (lattice snapping of the start heading) is corrected while driving off.
  if (!vertices.front().stop)
  {
    vertices.front().theta_in = vertices.front().theta_out;
  }
  return filletCorners(vertices);
}

std::vector<PathFollowerCritic::Vertex> PathFollowerCritic::filletCorners(const std::vector<Vertex>& vertices) const
{
  // Replace the kink of every driven-through in-place turn by a quadratic Bezier arc between the points
  // corner_fillet_length before and after it, so that the lateral and heading references agree on a path a
  // moving robot can actually follow. Neighbouring corners and stop points limit the fillet length.
  const double min_turn = 0.05;
  auto isCorner = [&](size_t i) {
    return !vertices[i].stop && i > 0 && i + 1 < vertices.size() &&
           std::fabs(angles::shortest_angular_distance(vertices[i].theta_in, vertices[i].theta_out)) >= min_turn;
  };
  auto isAnchor = [&](size_t i) { return vertices[i].stop || isCorner(i); };
  auto pointAt = [&](double s, size_t hint) {
    // Interpolate the polyline at arc length s, searching from the vertex index hint.
    size_t i = std::min(hint, vertices.size() - 2);
    while (i > 0 && vertices[i].s > s) --i;
    while (i + 2 < vertices.size() && vertices[i + 1].s < s) ++i;
    const Vertex& a = vertices[i];
    const Vertex& b = vertices[i + 1];
    double t = b.s > a.s ? std::max(0.0, std::min(1.0, (s - a.s) / (b.s - a.s))) : 0.0;
    Vertex v;
    v.x = a.x + t * (b.x - a.x);
    v.y = a.y + t * (b.y - a.y);
    v.theta_in = v.theta_out =
        angles::normalize_angle(a.theta_out + t * angles::shortest_angular_distance(a.theta_out, b.theta_in));
    v.forward = a.forward;
    return v;
  };

  std::vector<Vertex> out;
  out.reserve(vertices.size() + 8);
  double skip_until = -1.0;
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    const Vertex& corner = vertices[i];
    if (corner.s < skip_until)
    {
      continue;
    }
    if (!isCorner(i) || corner_fillet_length_ <= 0.0)
    {
      out.push_back(corner);
      continue;
    }
    size_t prev = i, next = i;
    while (prev > 0 && !isAnchor(prev - 1)) --prev;
    if (prev > 0) --prev;
    while (next + 1 < vertices.size() && !isAnchor(next + 1)) ++next;
    if (next + 1 < vertices.size()) ++next;
    // Two neighbouring corners share the space between them; a stop point or the plan end only needs a margin.
    double back = corner.s - vertices[prev].s, ahead = vertices[next].s - corner.s;
    double length = std::min({ corner_fillet_length_, isCorner(prev) ? 0.5 * back : back - 0.02,
                               isCorner(next) ? 0.5 * ahead : ahead - 0.02 });
    if (length < 0.02)
    {
      out.push_back(corner);
      continue;
    }
    while (!out.empty() && out.back().s > corner.s - length)
    {
      out.pop_back();
    }
    Vertex in = pointAt(corner.s - length, i);
    Vertex exit = pointAt(corner.s + length, i);
    out.push_back(in);
    double turn = angles::shortest_angular_distance(corner.theta_in, corner.theta_out);
    for (double t : { 0.25, 0.5, 0.75 })
    {
      Vertex v;
      double a = (1.0 - t) * (1.0 - t), b = 2.0 * t * (1.0 - t), c = t * t;
      v.x = a * in.x + b * corner.x + c * exit.x;
      v.y = a * in.y + b * corner.y + c * exit.y;
      v.theta_in = v.theta_out = angles::normalize_angle(corner.theta_in + t * turn);
      v.forward = corner.forward;
      out.push_back(v);
    }
    out.push_back(exit);
    skip_until = corner.s + length;
  }

  // Recompute arc lengths and segment directions for the smoothed polyline.
  out.front().s = 0.0;
  for (size_t i = 0; i + 1 < out.size(); ++i)
  {
    Vertex& from = out[i];
    Vertex& to = out[i + 1];
    double dx = to.x - from.x, dy = to.y - from.y;
    to.s = from.s + std::hypot(dx, dy);
    from.forward = dx * std::cos(from.theta_out) + dy * std::sin(from.theta_out) >= 0.0;
  }
  if (out.size() > 1)
  {
    out.back().forward = out[out.size() - 2].forward;
  }
  return out;
}

PathFollowerCritic::Projection PathFollowerCritic::project(double x, double y, size_t first_segment,
                                                           size_t last_segment, bool extend_ends) const
{
  Projection best;
  best.distance = kInfinity;
  if (vertices_.size() < 2)
  {
    best.x = vertices_.front().x;
    best.y = vertices_.front().y;
    best.distance = std::hypot(x - best.x, y - best.y);
    return best;
  }
  last_segment = std::min(last_segment, vertices_.size() - 2);
  for (size_t i = std::min(first_segment, last_segment); i <= last_segment; ++i)
  {
    const Vertex& a = vertices_[i];
    const Vertex& b = vertices_[i + 1];
    double dx = b.x - a.x, dy = b.y - a.y;
    double length_sq = dx * dx + dy * dy;
    double t = length_sq > 0.0 ? ((x - a.x) * dx + (y - a.y) * dy) / length_sq : 0.0;
    // Optionally treat the outer segments as rays so that overshooting the window is not a lateral error.
    if (!(extend_ends && i == first_segment))
    {
      t = std::max(0.0, t);
    }
    if (!(extend_ends && i == last_segment))
    {
      t = std::min(1.0, t);
    }
    double px = a.x + t * dx, py = a.y + t * dy;
    double distance = std::hypot(x - px, y - py);
    // Strictly better only: on ties keep the earliest segment so that loops in the plan don't skip ahead.
    if (distance < best.distance)
    {
      best.segment = i;
      best.t = t;
      best.distance = distance;
      best.s = a.s + t * (b.s - a.s);
      best.x = px;
      best.y = py;
    }
  }
  return best;
}

bool PathFollowerCritic::locateRobot(const geometry_msgs::Pose2D& pose, bool passed_valid, size_t passed_index)
{
  size_t num_segments = vertices_.size() - 1;
  if (num_segments == 0)
  {
    robot_ = project(pose.x, pose.y, 0, 0);
    have_passed_stop_ = false;
    have_turning_stop_ = false;
    have_progress_ = true;
    progress_x_ = robot_.x;
    progress_y_ = robot_.y;
    progress_segment_ = 0;
    return true;
  }

  bool continuing = false;
  size_t start = 0;
  if (have_progress_)
  {
    // Re-identify the previous progress point in this plan. Pruning shifts the vertex indices by a few per
    // cycle and the plan is re-transformed with some jitter, so among the segments (almost) containing the
    // point take the one closest to the previous index: plans that cross or overlap themselves (SBPL loops,
    // reversals) offer several equally close candidates.
    Projection previous;
    previous.distance = kInfinity;
    size_t best_index_distance = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < num_segments; ++i)
    {
      Projection candidate = project(progress_x_, progress_y_, i, i);
      size_t index_distance = i > progress_segment_ ? i - progress_segment_ : progress_segment_ - i;
      bool on_segment = candidate.distance <= 0.02;
      bool previous_on_segment = previous.distance <= 0.02;
      if ((on_segment && (!previous_on_segment || index_distance < best_index_distance)) ||
          (!on_segment && !previous_on_segment && candidate.distance < previous.distance))
      {
        previous = candidate;
        best_index_distance = index_distance;
      }
    }
    if (previous.distance <= plan_alignment_position_tolerance_)
    {
      start = previous.segment;
      continuing = true;
    }
  }

  if (continuing)
  {
    // Progress is confined to the current motion segment: never project back before the passed stop point and
    // never beyond the next one. Plans that fold back on themselves (reverse, turn, drive past the same spot)
    // would otherwise let the robot skip a stop point.
    if (passed_valid)
    {
      start = std::max(start, std::min(passed_index, num_segments - 1));
    }
    size_t last = start;
    while (last + 1 < num_segments && vertices_[last + 1].s - vertices_[start].s < search_window_ &&
           !(vertices_[last + 1].stop && !(passed_valid && last + 1 == passed_index)))
    {
      ++last;
    }
    robot_ = project(pose.x, pose.y, start, last);
  }
  else
  {
    // New plan or track lost: use the first local minimum of the distance to the plan that is within tolerance
    // (not the global minimum) so that loops in the plan don't make the robot skip ahead.
    have_passed_stop_ = false;
    have_turning_stop_ = false;
    robot_.distance = kInfinity;
    bool near_plan = false;
    for (size_t i = 0; i < num_segments; ++i)
    {
      Projection candidate = project(pose.x, pose.y, i, i);
      if (near_plan && candidate.distance >= robot_.distance)
      {
        break;
      }
      if (candidate.distance < robot_.distance)
      {
        robot_ = candidate;
      }
      near_plan = near_plan || candidate.distance <= plan_alignment_position_tolerance_;
    }
  }
  have_progress_ = true;
  progress_x_ = robot_.x;
  progress_y_ = robot_.y;
  progress_segment_ = robot_.segment;
  return true;
}

bool PathFollowerCritic::isContinuationOf(const std::vector<geometry_msgs::Pose2D>& plan,
                                          const std::vector<geometry_msgs::Pose2D>& previous)
{
  if (plan.empty() || previous.empty())
  {
    return false;
  }
  // The plan is re-transformed into the costmap frame every cycle; with a localization that publishes the
  // map->odom transform from noisy samples that moves the poses by several millimeters between cycles.
  const double tolerance = 0.02;
  const size_t run = std::min<size_t>(10, std::min(plan.size(), previous.size()));
  for (size_t offset = 0; offset + run <= previous.size(); ++offset)
  {
    bool match = true;
    for (size_t k = 0; k < run && match; ++k)
    {
      match = std::fabs(plan[k].x - previous[offset + k].x) < tolerance &&
              std::fabs(plan[k].y - previous[offset + k].y) < tolerance &&
              std::fabs(angles::shortest_angular_distance(plan[k].theta, previous[offset + k].theta)) < tolerance;
    }
    if (match)
    {
      return true;
    }
  }
  return false;
}

bool PathFollowerCritic::findStop(const geometry_msgs::Pose2D& stop, size_t& index) const
{
  for (size_t i = 0; i < vertices_.size(); ++i)
  {
    if (vertices_[i].stop && std::hypot(vertices_[i].x - stop.x, vertices_[i].y - stop.y) <= 0.02 &&
        std::fabs(angles::shortest_angular_distance(vertices_[i].theta_out, stop.theta)) <= 0.05)
    {
      index = i;
      return true;
    }
  }
  return false;
}

double PathFollowerCritic::pathHeading(const Projection& projection) const
{
  if (vertices_.size() < 2)
  {
    return vertices_.front().theta_out;
  }
  const Vertex& from = vertices_[projection.segment];
  const Vertex& to = vertices_[projection.segment + 1];
  double t = std::max(0.0, std::min(1.0, projection.t));
  return angles::normalize_angle(from.theta_out + t * angles::shortest_angular_distance(from.theta_out, to.theta_in));
}

bool PathFollowerCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan)
{
  prepared_ = false;
  turning_ = false;
  vertices_ = buildPolyline(global_plan, goal);
  if (vertices_.empty())
  {
    ROS_ERROR_NAMED("PathFollowerCritic", "The global plan was empty.");
    return false;
  }
  const size_t num_segments = vertices_.size() - 1;

  // DWB only prunes poses from the front of a plan and crops its tail, so the same plan still starts with a run
  // of poses found back to back in the previous one. Anything else is a new plan: restart the progress tracking
  // from its beginning instead of matching the old progress point, which a plan looping back near the robot
  // (typical for SBPL) would match to its later part.
  if (have_progress_ && !isContinuationOf(global_plan.poses, last_plan_))
  {
    ROS_DEBUG_NAMED("PathFollowerCritic", "New global plan received, restarting progress tracking.");
    have_progress_ = false;
    have_passed_stop_ = false;
    have_turning_stop_ = false;
  }
  last_plan_ = global_plan.poses;

  // Re-identify the last passed stop point in this (possibly pruned) plan.
  size_t passed_index = 0;
  bool passed_valid = have_passed_stop_ && findStop(passed_stop_, passed_index);
  have_passed_stop_ = passed_valid;
  locateRobot(pose, passed_valid, passed_index);
  passed_valid = have_passed_stop_;  // a new plan discards it

  auto nextStop = [&](size_t from, size_t& index) {
    for (size_t i = std::max(from, robot_.segment); i < vertices_.size(); ++i)
    {
      if (!vertices_[i].stop || (passed_valid && i == passed_index) ||
          vertices_[i].s < robot_.s - xy_local_goal_tolerance_)
      {
        continue;
      }
      index = i;
      return true;
    }
    return false;
  };

  // Driving direction of the segment ahead of the robot: the one leaving a stop point that was passed in this
  // very cycle (the robot still projects onto the segment arriving there), else the current segment.
  auto directionAhead = [&]() {
    size_t direction_segment = std::min(robot_.segment, num_segments > 0 ? num_segments - 1 : 0);
    if (passed_valid && robot_.segment < passed_index && passed_index < num_segments)
    {
      direction_segment = passed_index;
    }
    else if (robot_.t >= 1.0 - 1e-9 && robot_.segment + 1 < num_segments)
    {
      direction_segment = robot_.segment + 1;
    }
    return vertices_[direction_segment].forward ? 1.0 : -1.0;
  };
  speed_direction_ = directionAhead();

  size_t stop_index = 0;
  bool have_stop = false;
  if (have_turning_stop_ && findStop(turning_stop_, stop_index))
  {
    have_stop = true;
  }
  else
  {
    have_turning_stop_ = false;
    have_stop = nextStop(0, stop_index);
  }

  double distance_to_stop = kInfinity;
  while (have_stop)
  {
    const Vertex& stop = vertices_[stop_index];
    distance_to_stop = stop.s - robot_.s;
    bool is_final = stop_index + 1 == vertices_.size();
    double reach_tolerance = is_final ? xy_final_goal_tolerance_ : xy_local_goal_tolerance_;
    if (is_final && distance_to_stop < 0.1)
    {
      // The last bit of the plan is the jump from the lattice to the exact goal, which may be sideways. Only the
      // distance along the robot's heading can still be closed; a residual lateral offset is the goal checker's.
      double along = (stop.x - pose.x) * std::cos(pose.theta) + (stop.y - pose.y) * std::sin(pose.theta);
      distance_to_stop = std::min(distance_to_stop, std::max(0.0, speed_direction_ * along));
    }
    if (!have_turning_stop_ && distance_to_stop > reach_tolerance + 1e-3)
    {
      break;  // still driving towards it
    }
    double heading_error = angles::shortest_angular_distance(pose.theta, stop.theta_out);
    if (is_final || std::fabs(heading_error) > yaw_local_goal_tolerance_)
    {
      // Turn in place; the final pose is only released by the goal checker.
      turning_ = true;
      have_turning_stop_ = true;
      turning_stop_ = makePose(stop.x, stop.y, stop.theta_out);
      target_ = turning_stop_;
      desired_speed_ = 0.0;
      desired_rotation_ = desiredRotation(heading_error);
      break;
    }
    // Aligned with the outgoing direction: this stop point is done.
    have_passed_stop_ = true;
    passed_valid = true;
    passed_index = stop_index;
    passed_stop_ = makePose(stop.x, stop.y, stop.theta_out);
    have_turning_stop_ = false;
    have_stop = nextStop(stop_index + 1, stop_index);
    distance_to_stop = kInfinity;
  }

  if (!turning_)
  {
    speed_direction_ = directionAhead();
    double speed = desiredSpeed(distance_to_stop);
    // Curvature limit: the path ahead must be drivable with the angular velocity available, and the robot has to
    // be able to brake to that speed before getting there.
    if (curve_rotation_ > 0.0)
    {
      size_t last_segment = have_stop && stop_index > 0 ? stop_index - 1 : (num_segments > 0 ? num_segments - 1 : 0);
      for (size_t k = robot_.segment; k <= last_segment && k < num_segments; ++k)
      {
        const Vertex& from = vertices_[k];
        const Vertex& to = vertices_[k + 1];
        double ahead = std::max(0.0, from.s - robot_.s);
        if (ahead > curve_lookahead_)
        {
          break;
        }
        double length = to.s - from.s;
        if (length < 1e-3)
        {
          continue;
        }
        double curvature = std::fabs(angles::shortest_angular_distance(from.theta_out, to.theta_in)) / length;
        if (curvature < 1e-3)
        {
          continue;
        }
        double limit = curve_rotation_ / curvature;
        speed = std::min(speed, std::sqrt(limit * limit + 2.0 * decel_ * ahead));
      }
    }
    desired_speed_ = speed_direction_ * speed;
    desired_rotation_ = 0.0;
    if (have_stop)
    {
      target_ = makePose(vertices_[stop_index].x, vertices_[stop_index].y, vertices_[stop_index].theta_in);
    }
    else
    {
      target_ = makePose(vertices_.back().x, vertices_.back().y, vertices_.back().theta_in);
    }
    // Score deviations against the path from just behind the robot up to the next stop point. Segments before a
    // passed stop point are excluded: after a reversal or U-turn they overlap the ones ahead with opposite heading.
    window_first_ = robot_.segment > 0 ? robot_.segment - 1 : 0;
    if (passed_valid)
    {
      window_first_ = std::max(window_first_, passed_index);
    }
    window_last_ = have_stop && stop_index > 0 ? stop_index - 1 : (num_segments > 0 ? num_segments - 1 : 0);
    window_last_ = std::max(window_last_, window_first_);
  }

  ROS_DEBUG_NAMED("PathFollowerCritic",
                  "robot (%.3f, %.3f, %.2f) on segment %zu t=%.2f s=%.3f dist=%.3f | stop %s%zu d=%.3f | turning=%d "
                  "v_des=%.2f w_des=%.2f | passed=%s%zu | plan %zu poses, %zu vertices",
                  pose.x, pose.y, pose.theta, robot_.segment, robot_.t, robot_.s, robot_.distance,
                  have_stop ? "" : "none ", stop_index, distance_to_stop, turning_, desired_speed_, desired_rotation_,
                  passed_valid ? "" : "none ", passed_index, global_plan.poses.size(), vertices_.size());
  publishTarget(global_plan);
  prepared_ = true;
  return true;
}

double PathFollowerCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
  if (!prepared_ || traj.poses.empty())
  {
    throw nav_core2::IllegalTrajectoryException(name_, "No prepared path or empty trajectory.");
  }
  const nav_grid::NavGridInfo& info = costmap_->getInfo();
  for (const auto& pose : traj.poses)
  {
    // Center point check only; ObstacleFootprint does the full footprint.
    unsigned int cell_x, cell_y;
    if (worldToGridBounded(info, pose.x, pose.y, cell_x, cell_y) &&
        (*costmap_)(cell_x, cell_y) == nav_core2::Costmap::LETHAL_OBSTACLE)
    {
      throw nav_core2::IllegalTrajectoryException(name_, "Trajectory hits an obstacle.");
    }
  }

  const geometry_msgs::Pose2D& end = traj.poses.back();
  double speed_error = (traj.velocity.x - desired_speed_) / max_speed_;
  double score = speed_scale_ * speed_error * speed_error;

  if (turning_)
  {
    double rotation_error = (traj.velocity.theta - desired_rotation_) / max_rotation_;
    score += rotation_scale_ * rotation_error * rotation_error;
    // Hold (or creep onto) the stop point while turning.
    score += path_distance_scale_ * std::hypot(end.x - target_.x, end.y - target_.y) / info.resolution;
    return score;
  }

  // Lateral error: where the trajectory ends up plus any excursion beyond the initial offset (corner cutting).
  Projection first = project(traj.poses.front().x, traj.poses.front().y, window_first_, window_last_, true);
  Projection last = first;
  double max_deviation = first.distance;
  for (size_t i = 1; i < traj.poses.size(); ++i)
  {
    last = project(traj.poses[i].x, traj.poses[i].y, window_first_, window_last_, true);
    max_deviation = std::max(max_deviation, last.distance);
  }
  double lateral = last.distance + std::max(0.0, max_deviation - first.distance);
  score += path_distance_scale_ * lateral / info.resolution;

  double heading_error = angles::shortest_angular_distance(end.theta, pathHeading(last));
  score += heading_scale_ * heading_error * heading_error;
  return score;
}

void PathFollowerCritic::publishTarget(const nav_2d_msgs::Path2D& plan) const
{
  if (!intermediate_goal_pub_ || intermediate_goal_pub_.getNumSubscribers() == 0)
  {
    return;
  }
  geometry_msgs::PoseStamped msg;
  msg.header = plan.header;
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = target_.x;
  msg.pose.position.y = target_.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, target_.theta);
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  intermediate_goal_pub_.publish(msg);
}

}  // namespace mir_dwb_critics

PLUGINLIB_EXPORT_CLASS(mir_dwb_critics::PathFollowerCritic, dwb_local_planner::TrajectoryCritic)
