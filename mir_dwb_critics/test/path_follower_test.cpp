#include <gtest/gtest.h>
#include <mir_dwb_critics/braking_obstacle_footprint.h>
#include <mir_dwb_critics/path_follower.h>
#include <nav_core2/basic_costmap.h>
#include <nav_core2/exceptions.h>
#include <cmath>
#include <vector>

namespace
{
geometry_msgs::Pose2D pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::Pose2D p;
  p.x = x;
  p.y = y;
  p.theta = yaw;
  return p;
}

// Straight 0.4 m east, 90 degree turn in place (encoded like SBPL: several poses with identical XY), 0.4 m north.
std::vector<geometry_msgs::Pose2D> cornerPlan()
{
  return {pose(1, 1), pose(1.2, 1), pose(1.4, 1), pose(1.4, 1, 0.4), pose(1.4, 1, 0.8), pose(1.4, 1, 1.2),
          pose(1.4, 1, M_PI_2), pose(1.4, 1.2, M_PI_2), pose(1.4, 1.4, M_PI_2)};
}

class PathFollowerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle nh("/path_follower_regression");
    nh.setParam("max_vel_x", 0.8);
    nh.setParam("max_vel_theta", 1.0);
    nh.setParam("acc_lim_x", 1.5);
    nh.setParam("acc_lim_theta", 2.0);
    map = std::make_shared<nav_core2::BasicCostmap>();
    nav_grid::NavGridInfo info;
    info.width = info.height = 100;
    info.resolution = 0.05;
    map->setInfo(info);
    for (unsigned int y = 0; y < info.height; ++y)
      for (unsigned int x = 0; x < info.width; ++x)
        map->setValue(x, y, static_cast<unsigned char>(nav_core2::Costmap::FREE_SPACE));
    critic.initialize(nh, "PathFollower", map);
  }

  nav_2d_msgs::Path2D path(const std::vector<geometry_msgs::Pose2D>& points)
  {
    nav_2d_msgs::Path2D plan;
    plan.header.frame_id = "map";
    plan.poses = points;
    return plan;
  }

  void prepare(const std::vector<geometry_msgs::Pose2D>& points, const geometry_msgs::Pose2D& robot)
  {
    ASSERT_TRUE(critic.prepare(robot, nav_2d_msgs::Twist2D(), points.back(), path(points)));
  }

  double score(std::initializer_list<geometry_msgs::Pose2D> points, double v = 0.0, double w = 0.0)
  {
    dwb_msgs::Trajectory2D traj;
    traj.poses = points;
    traj.velocity.x = v;
    traj.velocity.theta = w;
    return critic.scoreTrajectory(traj);
  }

  mir_dwb_critics::PathFollowerCritic critic;
  std::shared_ptr<nav_core2::BasicCostmap> map;
};

TEST_F(PathFollowerTest, SplitsPlanIntoStopPoints)
{
  auto plan = cornerPlan();
  auto vertices = critic.buildPolyline(path(plan), plan.back());
  ASSERT_EQ(5u, vertices.size());
  EXPECT_FALSE(vertices[0].stop);
  EXPECT_FALSE(vertices[1].stop);
  EXPECT_TRUE(vertices[2].stop);  // 90 degree turn
  EXPECT_NEAR(0.0, vertices[2].theta_in, 1e-9);
  EXPECT_NEAR(M_PI_2, vertices[2].theta_out, 1e-9);
  EXPECT_FALSE(vertices[3].stop);
  EXPECT_TRUE(vertices[4].stop);  // final pose
  EXPECT_NEAR(0.8, vertices[4].s, 1e-9);

  // A plan cropped to the local costmap does not end with a stop point.
  vertices = critic.buildPolyline(path(plan), pose(5, 5));
  EXPECT_FALSE(vertices[4].stop);

  // A 22.5 degree lattice turn is driven through as an arc (filleted, hence more vertices, none of them a stop
  // except the final pose), a direction reversal is a stop point.
  vertices = critic.buildPolyline(path({pose(1, 1), pose(1.4, 1), pose(1.4, 1, 0.39), pose(1.8, 1.2, 0.39)}),
                                  pose(1.8, 1.2, 0.39));
  ASSERT_GT(vertices.size(), 3u);
  for (size_t i = 0; i + 1 < vertices.size(); ++i)
    EXPECT_FALSE(vertices[i].stop);
  EXPECT_TRUE(vertices.back().stop);
  vertices = critic.buildPolyline(path({pose(1, 1), pose(1.4, 1), pose(1.2, 1)}), pose(1.2, 1));
  ASSERT_EQ(3u, vertices.size());
  EXPECT_TRUE(vertices[0].forward);
  EXPECT_FALSE(vertices[1].forward);
  EXPECT_TRUE(vertices[1].stop);
}

TEST_F(PathFollowerTest, ApproachesTheFinalPoseCloserThanIntermediateStops)
{
  std::vector<geometry_msgs::Pose2D> plan = {pose(1, 1), pose(2, 1)};
  prepare(plan, pose(1, 1));
  prepare(plan, pose(1.96, 1));
  EXPECT_FALSE(critic.turning());  // 4 cm to go: an intermediate stop would count as reached, the final one not
  EXPECT_NEAR(3.0 * 0.04, critic.desiredVelocity(), 1e-9);
  prepare(plan, pose(1.985, 1));
  EXPECT_TRUE(critic.turning());

  // The exact goal appended to a lattice plan can be a few cm sideways: only the distance along the heading
  // counts for the final approach, the lateral rest is left to the goal checker instead of freezing the robot.
  std::vector<geometry_msgs::Pose2D> sideways = {pose(1, 1), pose(1.98, 1), pose(1.99, 1.03)};
  prepare(sideways, pose(1, 1));
  prepare(sideways, pose(1.92, 1.0));
  EXPECT_FALSE(critic.turning());
  EXPECT_NEAR(3.0 * 0.07, critic.desiredVelocity(), 1e-6);  // 7 cm along the heading, not 9 cm of arc length
  prepare(sideways, pose(1.985, 1.0));
  EXPECT_TRUE(critic.turning());
}

TEST_F(PathFollowerTest, CruisesAtMaxSpeedFarFromStopPoints)
{
  prepare({pose(1, 1), pose(4, 1)}, pose(1, 1));
  EXPECT_FALSE(critic.turning());
  EXPECT_DOUBLE_EQ(0.8, critic.desiredVelocity());
  EXPECT_LT(score({pose(1, 1), pose(1.64, 1)}, 0.8), score({pose(1, 1), pose(1.32, 1)}, 0.4));
}

TEST_F(PathFollowerTest, SlowsDownForCurvesAhead)
{
  // Straight, then a 1 m long arc of radius 0.5 m (curvature 2/m): with 0.9 rad/s budgeted, 0.45 m/s in the arc.
  std::vector<geometry_msgs::Pose2D> plan;
  for (int i = 0; i <= 20; ++i)
    plan.push_back(pose(1.0 + 0.1 * i, 1.0));
  for (int i = 1; i <= 20; ++i)
  {
    double a = 0.1 * i;  // arc length -> angle = a / 0.5
    plan.push_back(pose(3.0 + 0.5 * std::sin(2.0 * a), 1.0 + 0.5 * (1.0 - std::cos(2.0 * a)), 2.0 * a));
  }
  prepare(plan, pose(1.0, 1.0));
  EXPECT_DOUBLE_EQ(0.8, critic.desiredVelocity());  // the arc is more than 1 m away
  for (double x = 1.05; x < 2.85; x += 0.05)
    prepare(plan, pose(x, 1.0));
  EXPECT_LT(critic.desiredVelocity(), 0.8);  // braking towards the arc: sqrt(0.45^2 + 2 * 0.75 * 0.15) = 0.65
  EXPECT_NEAR(0.65, critic.desiredVelocity(), 0.02);
  for (double x = 2.85; x < 3.01; x += 0.05)
    prepare(plan, pose(x, 1.0));
  EXPECT_NEAR(0.45, critic.desiredVelocity(), 0.01);
}

TEST_F(PathFollowerTest, DeceleratesIntoStopPointAndTurnsThere)
{
  auto plan = cornerPlan();
  prepare(plan, pose(1.2, 1));
  EXPECT_FALSE(critic.turning());
  EXPECT_NEAR(std::sqrt(2.0 * 0.75 * 0.2), critic.desiredVelocity(), 1e-9);  // sqrt(2 * decel * distance)
  EXPECT_LT(score({pose(1.2, 1), pose(1.6, 1)}, 0.5), score({pose(1.2, 1), pose(1.84, 1)}, 0.8));

  prepare(plan, pose(1.34, 1));
  EXPECT_NEAR(3.0 * 0.06, critic.desiredVelocity(), 1e-9);  // approach_gain * distance

  prepare(plan, pose(1.38, 1));
  EXPECT_TRUE(critic.turning());
  EXPECT_DOUBLE_EQ(0.0, critic.desiredVelocity());
  EXPECT_NEAR(1.4, critic.target().x, 1e-9);
  EXPECT_NEAR(M_PI_2, critic.target().theta, 1e-9);
  // Far from the target heading the full angular velocity wins.
  EXPECT_LT(score({pose(1.38, 1), pose(1.38, 1, 0.8)}, 0.0, 1.0), score({pose(1.38, 1), pose(1.38, 1, 0.4)}, 0.0, 0.5));
  // Driving on while turning is penalized.
  EXPECT_LT(score({pose(1.38, 1), pose(1.38, 1, 0.8)}, 0.0, 1.0), score({pose(1.38, 1), pose(1.6, 1, 0.8)}, 0.3, 1.0));
}

TEST_F(PathFollowerTest, RotationProfileIsTrapezoidal)
{
  EXPECT_DOUBLE_EQ(1.0, critic.desiredRotation(M_PI_2));
  EXPECT_DOUBLE_EQ(-1.0, critic.desiredRotation(-M_PI_2));
  EXPECT_NEAR(std::sqrt(2.0 * 1.0 * 0.3), critic.desiredRotation(0.3), 1e-9);  // sqrt(2 * rot_decel * error)
  EXPECT_NEAR(4.0 * 0.05, critic.desiredRotation(0.05), 1e-9);                 // rotation_gain * error
  EXPECT_DOUBLE_EQ(0.0, critic.desiredRotation(0.0));
}

TEST_F(PathFollowerTest, TurnsInPlaceForSmallTurnsAtThePlanStart)
{
  std::vector<geometry_msgs::Pose2D> plan = {pose(1, 1), pose(1, 1, 0.39), pose(2.5, 1.6, 0.39)};
  prepare(plan, pose(1, 1));
  EXPECT_TRUE(critic.turning());
  EXPECT_NEAR(0.39, critic.target().theta, 1e-9);
  prepare(plan, pose(1, 1, 0.35));
  EXPECT_FALSE(critic.turning());
  EXPECT_DOUBLE_EQ(0.8, critic.desiredVelocity());
  // A tiny initial mismatch is just driven off.
  prepare({pose(1, 1), pose(1, 1, 0.1), pose(2.5, 1.15, 0.1)}, pose(1, 1));
  EXPECT_FALSE(critic.turning());
}

TEST_F(PathFollowerTest, RollsThroughSmallTurns)
{
  std::vector<geometry_msgs::Pose2D> plan = {pose(1, 1), pose(1.4, 1), pose(1.4, 1, 0.39), pose(3, 1.6, 0.39)};
  prepare(plan, pose(1.35, 1));
  EXPECT_FALSE(critic.turning());
  EXPECT_DOUBLE_EQ(0.8, critic.desiredVelocity());
  // The corner is replaced by a fillet: the polyline passes inside it by a few centimeters with a blended heading.
  auto vertices = critic.buildPolyline(path(plan), plan.back());
  double closest = 1e9, closest_heading = 0.0;
  for (const auto& v : vertices)
  {
    double dist = std::hypot(v.x - 1.4, v.y - 1.0);
    if (dist < closest)
    {
      closest = dist;
      closest_heading = v.theta_out;
    }
  }
  EXPECT_GT(closest, 0.01);
  EXPECT_LT(closest, 0.04);
  EXPECT_NEAR(0.195, closest_heading, 0.05);
  for (const auto& v : vertices)
    EXPECT_FALSE(v.stop && std::hypot(v.x - 1.4, v.y - 1.0) < 0.3);
  prepare(plan, pose(1.0, 1));
  EXPECT_LT(score({pose(1.0, 1), pose(1.4, 1, 0.195)}, 0.5), score({pose(1.0, 1), pose(1.4, 1, 0.0)}, 0.5));
  EXPECT_LT(score({pose(1.0, 1), pose(1.4, 1, 0.195)}, 0.5), score({pose(1.0, 1), pose(1.4, 1, 0.39)}, 0.5));
  // Well before the corner the reference is the segment heading.
  EXPECT_LT(score({pose(1.0, 1), pose(1.2, 1, 0.0)}, 0.25), score({pose(1.0, 1), pose(1.2, 1, 0.195)}, 0.25));
}

TEST_F(PathFollowerTest, AdvancesThroughInPlaceTurnAcrossPrunedPlans)
{
  auto plan = cornerPlan();
  prepare(plan, pose(1.4, 1, 0.0));
  EXPECT_TRUE(critic.turning());
  prepare(plan, pose(1.4, 1, 1.2));
  EXPECT_TRUE(critic.turning());
  EXPECT_GT(critic.desiredRotation(M_PI_2 - 1.2), 0.0);
  // Aligned with the outgoing segment: continue driving north towards the final pose.
  prepare(plan, pose(1.4, 1, M_PI_2 - 0.05));
  EXPECT_FALSE(critic.turning());
  EXPECT_NEAR(std::sqrt(2.0 * 0.75 * 0.4), critic.desiredVelocity(), 1e-9);
  EXPECT_NEAR(1.4, critic.target().y, 1e-9);
  // DWB prunes passed poses from the front of the plan; progress must survive that.
  plan.erase(plan.begin(), plan.begin() + 2);
  prepare(plan, pose(1.4, 1.02, M_PI_2));
  EXPECT_FALSE(critic.turning());
  EXPECT_GT(critic.desiredVelocity(), 0.5);
  EXPECT_LT(score({pose(1.4, 1.02, M_PI_2), pose(1.4, 1.3, M_PI_2)}, 0.7),
            score({pose(1.4, 1.02, M_PI_2), pose(1.4, 1.1, M_PI_2)}, 0.2));
}

TEST_F(PathFollowerTest, DoesNotSkipStopPointsWhenThePlanFoldsBack)
{
  // East 0.4 m, U-turn in place, back west over the same line: the outgoing segment overlaps the incoming one.
  std::vector<geometry_msgs::Pose2D> plan = {pose(1, 1), pose(1.4, 1), pose(1.4, 1, 1.0), pose(1.4, 1, 2.0),
                                             pose(1.4, 1, M_PI), pose(1.0, 1, M_PI), pose(0.6, 1, M_PI)};
  prepare(plan, pose(1.0, 1));
  EXPECT_FALSE(critic.turning());
  EXPECT_NEAR(1.4, critic.target().x, 1e-9);
  EXPECT_NEAR(0.0, critic.target().theta, 1e-9);
  prepare(plan, pose(1.2, 1.01));
  EXPECT_FALSE(critic.turning());
  EXPECT_NEAR(std::sqrt(2.0 * 0.75 * 0.2), critic.desiredVelocity(), 1e-3);
  prepare(plan, pose(1.38, 1.01));
  EXPECT_TRUE(critic.turning());
  EXPECT_NEAR(M_PI, std::fabs(critic.target().theta), 1e-9);
  // Turned around: drive west along the overlapping segment, scored against the outgoing heading.
  prepare(plan, pose(1.38, 1.01, M_PI));
  EXPECT_FALSE(critic.turning());
  EXPECT_GT(critic.desiredVelocity(), 0.5);
  EXPECT_NEAR(0.6, critic.target().x, 1e-9);
  EXPECT_LT(score({pose(1.38, 1.01, M_PI), pose(1.0, 1.0, M_PI)}, 0.5),
            score({pose(1.38, 1.01, M_PI), pose(1.0, 1.0, M_PI - 0.3)}, 0.5));
}

TEST_F(PathFollowerTest, RestartsProgressOnANewPlanThatLoopsBackToTheRobot)
{
  // First plan: straight east. The robot follows it for a while.
  std::vector<geometry_msgs::Pose2D> first = {pose(1, 1), pose(1.2, 1), pose(1.4, 1), pose(1.6, 1), pose(1.8, 1)};
  prepare(first, pose(1.0, 1));
  prepare(first, pose(1.5, 1));
  // Replan from the robot: north, then a loop that comes back right next to the start before continuing.
  std::vector<geometry_msgs::Pose2D> second = {pose(1.5, 1, M_PI_2),  pose(1.5, 1.5, M_PI_2), pose(1.5, 2.0, M_PI_2),
                                               pose(1.5, 2.4, M_PI_2), pose(1.5, 2.4, 0),      pose(1.9, 2.4, 0),
                                               pose(1.9, 2.4, -M_PI_2), pose(1.9, 1.02, -M_PI_2), pose(1.9, 1.02, M_PI),
                                               pose(1.5, 1.02, M_PI),  pose(1.1, 1.02, M_PI)};
  prepare(second, pose(1.5, 1, M_PI_2));
  EXPECT_FALSE(critic.turning());
  EXPECT_NEAR(1.5, critic.target().x, 1e-9);  // first stop point: the turn at (1.5, 2.4)
  EXPECT_NEAR(2.4, critic.target().y, 1e-9);
  EXPECT_DOUBLE_EQ(0.8, critic.desiredVelocity());

  // The same plan pruned at the front (as DWB does once poses are 1 m behind) is not a new plan.
  prepare(second, pose(1.5, 1.6, M_PI_2));
  second.erase(second.begin());
  prepare(second, pose(1.5, 2.1, M_PI_2));
  EXPECT_NEAR(2.4, critic.target().y, 1e-9);
  EXPECT_NEAR(std::sqrt(2.0 * 0.75 * 0.3), critic.desiredVelocity(), 1e-6);
}

TEST_F(PathFollowerTest, KeepsProgressOnTheRightPassOfAnOverlappingPlan)
{
  // East, U-turn, back west over the same line, U-turn, east again: three passes over the same segment.
  std::vector<geometry_msgs::Pose2D> plan = {pose(1, 1), pose(1.5, 1), pose(2, 1), pose(2, 1, M_PI),
                                             pose(1.5, 1, M_PI), pose(1, 1, M_PI), pose(1, 1, 0),
                                             pose(1.5, 1, 0), pose(2, 1, 0), pose(2.4, 1, 0)};
  prepare(plan, pose(1.0, 1));
  for (double x = 1.05; x < 1.95; x += 0.05)
  {
    prepare(plan, pose(x, 1.0));
    EXPECT_FALSE(critic.turning()) << "x=" << x;
    EXPECT_GT(critic.desiredVelocity(), 0.0) << "x=" << x;
    EXPECT_NEAR(2.0, critic.target().x, 1e-9) << "x=" << x;  // first U-turn stays the target on the first pass
  }
}

TEST_F(PathFollowerTest, ReversesAlongBackwardSegments)
{
  // Arriving at the reversal point slightly short of it: reverse immediately, no forward glitch.
  std::vector<geometry_msgs::Pose2D> plan = {pose(1, 1), pose(1.4, 1), pose(1.2, 1)};
  prepare(plan, pose(1.0, 1));
  prepare(plan, pose(1.37, 1));
  EXPECT_FALSE(critic.turning());
  EXPECT_LT(critic.desiredVelocity(), 0.0);

  prepare({pose(1, 1), pose(1.4, 1), pose(1.2, 1)}, pose(1.4, 1));
  EXPECT_FALSE(critic.turning());
  EXPECT_NEAR(-std::sqrt(2.0 * 0.75 * 0.2), critic.desiredVelocity(), 1e-9);
  EXPECT_LT(score({pose(1.4, 1), pose(1.2, 1)}, -0.5), score({pose(1.4, 1), pose(1.6, 1)}, 0.5));
}

TEST_F(PathFollowerTest, PenalizesExcursionEvenWhenEndpointReturnsToPath)
{
  prepare({pose(1, 1), pose(4, 1)}, pose(1, 1));
  EXPECT_GT(score({pose(1, 1), pose(1.2, 1.2), pose(1.4, 1)}, 0.5), score({pose(1, 1), pose(1.2, 1), pose(1.4, 1)}, 0.5));
}

TEST_F(PathFollowerTest, DoesNotRewardDenserTrajectorySampling)
{
  prepare({pose(1, 1), pose(4, 1)}, pose(1, 1));
  EXPECT_DOUBLE_EQ(score({pose(1, 1), pose(1.2, 1.1), pose(1.4, 1)}, 0.5),
                   score({pose(1, 1), pose(1.2, 1.1), pose(1.2, 1.1), pose(1.4, 1)}, 0.5));
}

TEST_F(PathFollowerTest, PrefersAlignedHeadingAtTheEndpoint)
{
  prepare({pose(1, 1), pose(4, 1)}, pose(1, 1));
  EXPECT_LT(score({pose(1, 1), pose(1.4, 1, 0.0)}, 0.5), score({pose(1, 1), pose(1.4, 1, 0.3)}, 0.5));
}

TEST_F(PathFollowerTest, RejectsEmptyTrajectoryAndFailedPreparation)
{
  prepare({pose(1, 1), pose(2, 1)}, pose(1, 1));
  EXPECT_THROW(score({}), nav_core2::IllegalTrajectoryException);
  EXPECT_FALSE(critic.prepare(pose(1, 1), nav_2d_msgs::Twist2D(), pose(2, 1), nav_2d_msgs::Path2D()));
  EXPECT_THROW(score({pose(1, 1)}), nav_core2::IllegalTrajectoryException);
}

TEST_F(PathFollowerTest, KeepsObstacleRejection)
{
  map->setValue(28, 20, static_cast<unsigned char>(nav_core2::Costmap::LETHAL_OBSTACLE));
  prepare({pose(1, 1), pose(2, 1)}, pose(1, 1));
  EXPECT_THROW(score({pose(1, 1), pose(1.425, 1.025)}), nav_core2::IllegalTrajectoryException);
}
TEST_F(PathFollowerTest, BrakingObstacleFootprintChecksOnlyTheStoppingDistance)
{
  ros::NodeHandle nh("/path_follower_regression");
  XmlRpc::XmlRpcValue footprint;
  footprint.setSize(4);
  const double corners[4][2] = {{0.2, -0.1}, {0.2, 0.1}, {-0.2, 0.1}, {-0.2, -0.1}};
  for (int i = 0; i < 4; ++i)
  {
    footprint[i].setSize(2);
    footprint[i][0] = corners[i][0];
    footprint[i][1] = corners[i][1];
  }
  nh.setParam("footprint", footprint);
  nh.setParam("ObstacleFootprint/reaction_time", 0.1);
  mir_dwb_critics::BrakingObstacleFootprintCritic obstacle;
  obstacle.initialize(nh, "ObstacleFootprint", map);
  // Wall at x = 2.0 (cells 40..41), the robot drives east along y = 1 from x = 1.
  for (unsigned int y = 0; y < 100; ++y)
    for (unsigned int x = 40; x < 42; ++x)
      map->setValue(x, y, static_cast<unsigned char>(nav_core2::Costmap::LETHAL_OBSTACLE));
  ASSERT_TRUE(obstacle.prepare(pose(1, 1), nav_2d_msgs::Twist2D(), pose(3, 1), path({pose(1, 1), pose(3, 1)})));

  auto rollout = [](double v) {
    dwb_msgs::Trajectory2D traj;
    traj.velocity.x = v;
    for (int i = 0; i <= 8; ++i)
    {
      traj.poses.push_back(pose(1.0 + v * 0.1 * i, 1));
      traj.time_offsets.push_back(ros::Duration(0.1 * i));
    }
    return traj;
  };
  // At 0.8 m/s the 0.8 s rollout ends at x = 1.64 with its front at 1.84: the whole rollout is free.
  EXPECT_NO_THROW(obstacle.scoreTrajectory(rollout(0.8)));
  // At 1.2 m/s the rollout front reaches 2.16 (into the wall), but the braking distance (1.2 / 3 + 0.1 = 0.5 s of
  // rollout, front at 1.8) is free -> legal. At 1.6 m/s the wall is within the braking distance -> illegal.
  EXPECT_NO_THROW(obstacle.scoreTrajectory(rollout(1.2)));
  EXPECT_THROW(obstacle.scoreTrajectory(rollout(1.6)), nav_core2::IllegalTrajectoryException);
  auto slow = rollout(0.3);
  slow.poses.back().x = 1.9;  // fictitious constant-velocity tail reaching the wall after the braking horizon
  EXPECT_NO_THROW(obstacle.scoreTrajectory(slow));
  slow.poses[1].x = 1.9;  // within the horizon it counts
  EXPECT_THROW(obstacle.scoreTrajectory(slow), nav_core2::IllegalTrajectoryException);
}
TEST_F(PathFollowerTest, BrakingObstacleFootprintInsetAndEscape)
{
  ros::NodeHandle nh("/path_follower_regression");
  XmlRpc::XmlRpcValue footprint;
  footprint.setSize(4);
  const double corners[4][2] = {{0.2, -0.1}, {0.2, 0.1}, {-0.2, 0.1}, {-0.2, -0.1}};
  for (int i = 0; i < 4; ++i)
  {
    footprint[i].setSize(2);
    footprint[i][0] = corners[i][0];
    footprint[i][1] = corners[i][1];
  }
  nh.setParam("footprint", footprint);
  nh.setParam("Inset/footprint_inset", 0.02);
  nh.setParam("Inset/escape_current_collision", true);
  mir_dwb_critics::BrakingObstacleFootprintCritic obstacle;
  obstacle.initialize(nh, "Inset", map);
  // Wall at x = 2.0 (cells 40..41). Front of the exact footprint at x = 2.01 overlaps the wall cell by 1 cm,
  // the 2 cm inset one does not.
  for (unsigned int y = 0; y < 100; ++y)
    for (unsigned int x = 40; x < 42; ++x)
      map->setValue(x, y, static_cast<unsigned char>(nav_core2::Costmap::LETHAL_OBSTACLE));
  auto standing = [](double x) {
    dwb_msgs::Trajectory2D traj;
    for (int i = 0; i <= 3; ++i)
    {
      traj.poses.push_back(pose(x, 1));
      traj.time_offsets.push_back(ros::Duration(0.1 * i));
    }
    return traj;
  };
  ASSERT_TRUE(obstacle.prepare(pose(1.81, 1), nav_2d_msgs::Twist2D(), pose(3, 1), path({pose(1, 1), pose(3, 1)})));
  EXPECT_FALSE(obstacle.currentPoseBlocked());
  EXPECT_NO_THROW(obstacle.scoreTrajectory(standing(1.81)));
  // Robot standing with its (inset) front inside the wall: the current pose is blocked, standing still and driving
  // on stay illegal, backing out is legal.
  ASSERT_TRUE(obstacle.prepare(pose(1.83, 1), nav_2d_msgs::Twist2D(), pose(3, 1), path({pose(1, 1), pose(3, 1)})));
  EXPECT_TRUE(obstacle.currentPoseBlocked());
  EXPECT_THROW(obstacle.scoreTrajectory(standing(1.83)), nav_core2::IllegalTrajectoryException);
  dwb_msgs::Trajectory2D forward = standing(1.83), backward = standing(1.83);
  for (int i = 1; i <= 3; ++i)
  {
    forward.poses[i].x += 0.05 * i;
    backward.poses[i].x -= 0.05 * i;
  }
  forward.velocity.x = 0.2;
  backward.velocity.x = -0.2;
  EXPECT_THROW(obstacle.scoreTrajectory(forward), nav_core2::IllegalTrajectoryException);
  EXPECT_NO_THROW(obstacle.scoreTrajectory(backward));
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_follower_regression");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
