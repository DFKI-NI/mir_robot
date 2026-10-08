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
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace mir_navigation
{
/**
 * @brief SBPLLatticePlanner whose plans end at the requested goal pose.
 *
 * sbpl_lattice_planner leaves out the last intermediate pose of every motion primitive (it coincides with the
 * first pose of the next one), so its plans end one sample short of the final primitive, and its goal is snapped
 * to the lattice (5 cm cells, 22.5 degree headings). The DWB local planner steers to the last pose of the plan,
 * which therefore ended up to 4 cm and 0.075 rad away from where the robot was sent. The last motion of the plan
 * (up to goal_blend_distance of it) is blended into the exact goal, so that a differential drive robot can
 * correct the offset while still moving instead of facing a sideways jump at the very end.
 */
class SBPLLatticePlannerExactGoal : public sbpl_lattice_planner::SBPLLatticePlanner
{
public:
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override
  {
    // Keep reading the SBPL parameters from the namespace the config files use.
    sbpl_lattice_planner::SBPLLatticePlanner::initialize("SBPLLatticePlanner", costmap_ros);
    ros::NodeHandle private_nh("~/SBPLLatticePlanner");
    private_nh.param("goal_blend_distance", goal_blend_distance_, 0.3);
  }

  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override
  {
    if (!sbpl_lattice_planner::SBPLLatticePlanner::makePlan(start, goal, plan) || plan.empty())
    {
      return false;
    }
    blendIntoGoal(goal, plan);
    return true;
  }

  void blendIntoGoal(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) const
  {
    // The last motion run of the plan: walk back over consecutive poses that actually move (an in-place turn has
    // identical positions) until goal_blend_distance of path is covered.
    std::vector<double> distances(plan.size(), 0.0);
    size_t tail = plan.size() - 1;
    double length = 0.0;
    while (tail > 0)
    {
      const auto& a = plan[tail - 1].pose.position;
      const auto& b = plan[tail].pose.position;
      double step = std::hypot(b.x - a.x, b.y - a.y);
      if (step < 1e-6 || length + step > goal_blend_distance_)
      {
        break;
      }
      length += step;
      --tail;
      distances[tail] = length;  // path length from this pose to the plan end
    }
    const auto& end = plan.back().pose;
    double dx = goal.pose.position.x - end.position.x, dy = goal.pose.position.y - end.position.y;
    double dyaw = angles::shortest_angular_distance(tf2::getYaw(end.orientation), tf2::getYaw(goal.pose.orientation));
    for (size_t i = tail; i + 1 < plan.size(); ++i)
    {
      double w = length > 0.0 ? 1.0 - distances[i] / length : 0.0;
      auto& pose = plan[i].pose;
      pose.position.x += w * dx;
      pose.position.y += w * dy;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, tf2::getYaw(pose.orientation) + w * dyaw);
      pose.orientation = tf2::toMsg(q);
    }
    plan.back().pose = goal.pose;
  }

private:
  double goal_blend_distance_;
};

}  // namespace mir_navigation

PLUGINLIB_EXPORT_CLASS(mir_navigation::SBPLLatticePlannerExactGoal, nav_core::BaseGlobalPlanner)
