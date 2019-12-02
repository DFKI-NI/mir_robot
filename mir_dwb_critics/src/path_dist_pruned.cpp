#include <nav_2d_utils/path_ops.h>
#include <pluginlib/class_list_macros.h>

#include "mir_dwb_critics/path_dist_pruned.h"

namespace mir_dwb_critics
{


bool PathDistPrunedCritic::prepare(
        const geometry_msgs::Pose2D& pose,
        const nav_2d_msgs::Twist2D& vel,
        const geometry_msgs::Pose2D& goal,
        const nav_2d_msgs::Path2D& global_plan)
{

    const nav_core2::Costmap &costmap = *costmap_;
    const nav_grid::NavGridInfo &info = costmap.getInfo();

    auto plan = nav_2d_utils::adjustPlanResolution(global_plan, info.resolution).poses;
    

    // --- stolen from PathProgressCritic::getGoalPose ---
    // find the "start pose", i.e. the pose on the plan closest to the robot that is also on the local map
    unsigned int start_index = 0;
    double distance_to_start = std::numeric_limits<double>::infinity();
    bool started_path = false;
    for (unsigned int i = 0; i < plan.size(); i++) {
        double g_x = plan[i].x;
        double g_y = plan[i].y;
        unsigned int map_x, map_y;
        if (worldToGridBounded(info, g_x, g_y, map_x, map_y)
                && costmap(map_x, map_y) != nav_core2::Costmap::NO_INFORMATION) {
            // Still on the costmap. Continue.
            double distance = nav_2d_utils::poseDistance(plan[i], pose);
            if (distance_to_start > distance) {
                start_index = i;
                distance_to_start = distance;
                started_path = true;
            } else {
                // Plan is going away from the robot again. It's possible that it comes back and we would find a pose that's
                // even closer to the robot, but then we would skip over parts of the plan.
                break;
            }
        } else if (started_path) {
            // Off the costmap after being on the costmap.
            break;
        }
        // else, we have not yet found a point on the costmap, so we just continue
    }

    if (!started_path) {
        ROS_ERROR_NAMED("PathProgressCritic", "None of the points of the global plan were in the local costmap.");
        return false;
    }
    // ---------------------------------------------------
    
    // remove the first part of the path, everything before start_index, to
    // disregard that part in the PathDistCritic implementation.
    nav_2d_msgs::Path2D global_plan_pruned;
    global_plan_pruned.header = global_plan.header;
    global_plan_pruned.poses = std::vector<geometry_msgs::Pose2D>(
                                    global_plan.poses.begin() + start_index,
                                    global_plan.poses.end());

    return dwb_critics::PathDistCritic::prepare(pose, vel, goal, global_plan_pruned);
}


}

PLUGINLIB_EXPORT_CLASS(mir_dwb_critics::PathDistPrunedCritic, dwb_local_planner::TrajectoryCritic)
