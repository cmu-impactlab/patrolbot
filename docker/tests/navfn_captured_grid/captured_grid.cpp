#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <utility>
#include <vector>
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"

// Supply the captured grid without starting a live costmap or sensor graph.
class CapturedPlanner : public nav2_navfn_planner::NavfnPlanner
{
public:
  using nav2_navfn_planner::NavfnPlanner::worldToMap;

  explicit CapturedPlanner(nav2_costmap_2d::Costmap2D & grid)
  {
    costmap_ = &grid;
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    global_frame_ = "map";
    tolerance_ = .5;
    use_astar_ = false;
    allow_unknown_ = true;
    use_final_approach_orientation_ = false;
    planner_ = std::make_unique<nav2_navfn_planner::NavFn>(
      grid.getSizeInCellsX(), grid.getSizeInCellsY());
  }
};

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.w = 1.;
  return result;
}

void check_goal(
  const char * filename, const char * label,
  const geometry_msgs::msg::PoseStamped & supplied_start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & trapped_starts)
{
  std::ifstream file(filename, std::ios::binary);
  assert(file.is_open());
  const std::vector<unsigned char> data{
    std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
  assert(data.size() == 4235U * 3289U);
  nav2_costmap_2d::Costmap2D grid(4235, 3289, .05000000074505806, 0., 0.);
  CapturedPlanner planner(grid);
  std::copy(data.begin(), data.end(), grid.getCharMap());
  const auto path = planner.createPlan(supplied_start, goal, [] {return false;});
  assert(path.poses.size() > 2);
  assert(path.header.frame_id == "map");
  assert(std::hypot(path.poses.back().pose.position.x - goal.pose.position.x,
    path.poses.back().pose.position.y - goal.pose.position.y) <= .5);
  std::copy(data.begin(), data.end(), grid.getCharMap());
  const auto same_position_path = planner.createPlan(
    supplied_start, supplied_start, [] {return false;});
  assert(same_position_path.poses.size() == 1);
  for (const auto & start : trapped_starts) {
    std::copy(data.begin(), data.end(), grid.getCharMap());
    unsigned int mx, my;
    assert(planner.worldToMap(start.pose.position.x, start.pose.position.y, mx, my));
    const auto original_cost = grid.getCost(mx, my);
    assert(original_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    for (const auto & target : {goal, start}) {
      bool wall_failed = false;
      try {
        planner.createPlan(start, target, [] {return false;});
      } catch (const nav2_core::StartOccupied & error) {
        wall_failed = true;
        assert(error.what()[0] != '\0');
      }
      assert(wall_failed);
      assert(grid.getCost(mx, my) == original_cost);
    }
  }
  std::cout << "ACCEPTED " << label << " supplied_start_path_points=" << path.poses.size()
            << " trapped_starts=" << trapped_starts.size()
            << " wall_start=START_OCCUPIED physical_acceptance=false\n";
}

void check_usable_exit()
{
  for (const auto exit_cost : {nav2_costmap_2d::FREE_SPACE, nav2_costmap_2d::NO_INFORMATION}) {
    nav2_costmap_2d::Costmap2D grid(20, 20, .1, 0., 0., nav2_costmap_2d::FREE_SPACE);
    CapturedPlanner planner(grid);
    const std::pair<unsigned int, unsigned int> blocked[] = {
      {10, 10}, {9, 10}, {11, 10}, {10, 9}, {10, 11}};
    for (const auto & cell : blocked) {
      grid.setCost(cell.first, cell.second, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
    grid.setCost(11, 10, exit_cost);
    const auto path = planner.createPlan(pose(1., 1.), pose(1.5, 1.), [] {return false;});
    assert(path.poses.size() > 2);
    assert(grid.getCost(10, 10) == nav2_costmap_2d::FREE_SPACE);
  }
  std::cout << "ACCEPTED occupied_start_with_usable_exit free_and_allowed_unknown physical_acceptance=false\n";
}

int main(int argc, char ** argv)
{
  assert(std::getenv("PATROLBOT_ISOLATED_TEST") != nullptr);
  assert(argc == 3);
  check_goal(argv[1], "20261013-audit273", pose(94.73, 140.93), pose(94.09, 139.08),
    {pose(94.52, 141.95)});
  check_goal(argv[2], "20261014-audit346", pose(93.72, 141.28), pose(95.16, 133.66),
    {pose(93.73, 142.01)});
  check_goal(argv[2], "20261014-audit348", pose(93.72, 141.28), pose(95.13, 141.34),
    {pose(93.55, 141.96), pose(93.57, 141.99)});
  check_usable_exit();
}
