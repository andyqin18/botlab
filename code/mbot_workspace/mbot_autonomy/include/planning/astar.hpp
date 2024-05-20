#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <utils/grid_utils.hpp>

typedef Point<int> cell_t;

/**
 * SearchParams defines the parameters to use when searching for a path. See associated comments for
 * details
 */
struct SearchParams {
  double minDistanceToObstacle;  ///< The minimum distance a robot can be from an obstacle before
                                 ///< a collision occurs

  double
      maxDistanceWithCost;  ///< The maximum distance from an obstacle that has an associated cost.
                            ///< The planned path will attempt to stay at least this distance from
                            ///< obstacles unless it must travel closer to actually find a path

  double distanceCostExponent;  ///< The exponent to apply to the distance cost, whose function is:
                                ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                ///< for cellDistance > minDistanceToObstacle && cellDistance <
                                ///< maxDistanceWithCost
};

struct Edge;

double h_cost(const cell_t& from, const cell_t& goal);
std::vector<Edge> expand_cell(const cell_t& cell,
                              const ObstacleDistanceGrid& distances,
                              const SearchParams& params);
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(const std::vector<cell_t>& cells,
                                                       const ObstacleDistanceGrid& distances);
std::vector<cell_t> prune_path(const std::vector<cell_t>& path);

/**
 * search_for_path uses an A* search to find a path from the start to goal poses. The search assumes
 * a circular robot
 *
 * \param    start           Starting pose of the robot
 * \param    goal            Desired goal pose of the robot
 * \param    distances       Distance to the nearest obstacle for each cell in the grid
 * \param    params          Parameters specifying the behavior of the A* search
 * \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with
 * just the initial pose is returned, per the path2D_t specification.
 */
mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                        mbot_lcm_msgs::pose2D_t goal,
                                        const ObstacleDistanceGrid& distances,
                                        const SearchParams& params);

#endif  // PLANNING_ASTAR_HPP
