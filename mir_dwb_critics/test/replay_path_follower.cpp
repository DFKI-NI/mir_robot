// Replays recorded robot poses against a recorded global plan through the PathFollowerCritic and prints its
// decisions, mimicking DWB's plan pruning. Debug tool, not a test.
// Usage: replay_path_follower <file>   (file: "plan N" + N lines "x y theta", "robot M" + M lines "t x y theta")
#include <mir_dwb_critics/path_follower.h>
#include <nav_core2/basic_costmap.h>
#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "replay_path_follower");
  if (argc < 2)
  {
    std::cerr << "usage: replay_path_follower <file>" << std::endl;
    return 1;
  }
  std::ifstream in(argv[1]);
  std::string word;
  size_t n;
  in >> word >> n;
  nav_2d_msgs::Path2D plan;
  plan.header.frame_id = "map";
  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  for (size_t i = 0; i < n; ++i)
  {
    geometry_msgs::Pose2D p;
    in >> p.x >> p.y >> p.theta;
    plan.poses.push_back(p);
    min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
  }
  in >> word >> n;
  std::vector<std::pair<double, geometry_msgs::Pose2D>> robot(n);
  for (size_t i = 0; i < n; ++i)
  {
    in >> robot[i].first >> robot[i].second.x >> robot[i].second.y >> robot[i].second.theta;
  }

  ros::NodeHandle nh("/replay_path_follower");
  auto map = std::make_shared<nav_core2::BasicCostmap>();
  nav_grid::NavGridInfo info;
  info.resolution = 0.05;
  info.origin_x = min_x - 3.0;
  info.origin_y = min_y - 3.0;
  info.width = static_cast<unsigned int>((max_x - min_x + 6.0) / info.resolution);
  info.height = static_cast<unsigned int>((max_y - min_y + 6.0) / info.resolution);
  map->setInfo(info);
  for (unsigned int y = 0; y < info.height; ++y)
    for (unsigned int x = 0; x < info.width; ++x)
      map->setValue(x, y, 0);
  mir_dwb_critics::PathFollowerCritic critic;
  critic.initialize(nh, "PathFollower", map);

  nav_2d_msgs::Path2D global_plan = plan;
  for (const auto& entry : robot)
  {
    const geometry_msgs::Pose2D& pose = entry.second;
    // DWB pruning: drop leading poses until one is within prune_distance (1 m) of the robot.
    while (global_plan.poses.size() > 1 && std::hypot(global_plan.poses.front().x - pose.x,
                                                      global_plan.poses.front().y - pose.y) >= 1.0)
    {
      global_plan.poses.erase(global_plan.poses.begin());
    }
    // DWB cropping: keep poses within the local costmap radius (2 m).
    nav_2d_msgs::Path2D transformed = global_plan;
    transformed.poses.clear();
    for (const auto& p : global_plan.poses)
    {
      bool far = std::hypot(p.x - pose.x, p.y - pose.y) > 2.0;
      if (far && transformed.poses.empty()) continue;
      transformed.poses.push_back(p);
      if (far) break;
    }
    bool ok = critic.prepare(pose, nav_2d_msgs::Twist2D(), plan.poses.back(), transformed);
    auto target = critic.target();
    std::printf("t=%6.2f pose=(%.3f,%.3f,%+.2f) ok=%d turning=%d v_des=%+.2f target=(%.3f,%.3f,%+.2f) plan=%zu\n",
                entry.first, pose.x, pose.y, pose.theta, ok, critic.turning(), critic.desiredVelocity(), target.x,
                target.y, target.theta, transformed.poses.size());
  }
  return 0;
}
