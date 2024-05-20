#include <planning/astar.hpp>

#include <algorithm>
#include <chrono>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std::chrono;

// A directed edge of a graph. The source cell is omitted in this struct and
// should instead be tracked by a higher-level data structure.
struct Edge {
  Edge(const cell_t& cell, double w) : dest(cell), weight(w) {}

  bool operator<(const Edge& other) const { return weight < other.weight; }

  bool operator>(const Edge& other) const { return weight > other.weight; }

  cell_t dest;
  double weight;
};

class Point32Hash {
 public:
  size_t operator()(const Point<int32_t>& p) const {
    // Does this still work if p is not aligned on an 8-byte boundary?
    const uint64_t* u64 = reinterpret_cast<const uint64_t*>(&p);
    return std::hash<uint64_t>{}(*u64);
  }
};

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                        mbot_lcm_msgs::pose2D_t goal,
                                        const ObstacleDistanceGrid& distances,
                                        const SearchParams& params) {
  const cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
  const cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);

  ////////////////// TODO: Implement your A* search here //////////////////////////
  mbot_lcm_msgs::path2D_t path;
  path.utime = start.utime;

  // The caller checks whether the goal pose is valid.

  std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> frontier;  // g + h

  // TODO: these can probably all be consolidated into one map
  std::unordered_map<cell_t, double, Point32Hash> min_gcost_to_v;  // g only
  std::unordered_map<cell_t, cell_t, Point32Hash> path_parent;
  std::unordered_set<cell_t, Point32Hash> opt_visited;

  min_gcost_to_v[startCell] = 0;
  frontier.emplace(startCell, h_cost(startCell, goalCell));

  bool warn_about_unbounded_growth = true;

  while (!frontier.empty()) {
    Edge min_edge = frontier.top();
    frontier.pop();

    // This reduces the number of vertices expanded when no path
    // exists between the start and goal. (TODO: why?)
    if (opt_visited.count(min_edge.dest) == 1) {
      continue;
    }
    opt_visited.insert(min_edge.dest);

    if (min_edge.dest == goalCell) {
      break;
    }

    for (const Edge& e : expand_cell(min_edge.dest, distances, params)) {
      const cell_t& neighbor = e.dest;
      const double neighbor_gcost = min_gcost_to_v[min_edge.dest] + e.weight;

      auto gcost_lookup_it = min_gcost_to_v.find(neighbor);
      if (gcost_lookup_it == min_gcost_to_v.cend() || neighbor_gcost < gcost_lookup_it->second) {
        min_gcost_to_v[neighbor] = neighbor_gcost;
        path_parent[neighbor] = min_edge.dest;
        frontier.emplace(neighbor, neighbor_gcost + h_cost(neighbor, goalCell));
      }
    }

    if (warn_about_unbounded_growth && frontier.size() > 10000) {
      std::cerr << "WARNING: A* is growing without bounds\n";
      warn_about_unbounded_growth = false;
    }
  }

  // Reconstruct the path
  if (min_gcost_to_v.find(goalCell) == min_gcost_to_v.cend()) {
    std::cerr << "[A*] Didn't find a path!\n";
  } else {
    std::cout << "[A*] cost to goal: " << min_gcost_to_v[goalCell] << std::endl;

    cell_t current_cell = goalCell;
    while (current_cell != startCell) {
      // TODO: is this the correct utime to use?


     const Point<double> global_point = grid_position_to_global_position(Point<double>(current_cell.x, current_cell.y), distances);


      path.path.push_back({start.utime, global_point.x,
                           global_point.y * distances.metersPerCell(), 0});
      current_cell = path_parent[current_cell];
    }
    std::reverse(path.path.begin(), path.path.end());

    // Replace the last pose with the goal pose
    path.path.pop_back();
    path.path.push_back(goal);
  }
  path.path_length = path.path.size();

  return path;
}

double h_cost(const cell_t& from, const cell_t& goal) {
  // 8-connected heuristic
  const int dx = std::abs(from.x - goal.y);
  const int dy = std::abs(from.y - goal.y);

  return std::min(dx, dy) * M_SQRT2 + std::abs(dx - dy);
}

std::vector<Edge> expand_cell(const cell_t& cell,
                              const ObstacleDistanceGrid& distances,
                              const SearchParams& params) {
  // Weights are in cell units.
  static const std::array<Edge, 8> expansion_dirs{{
      {cell_t(1, 0), 1.0},
      {cell_t(0, 1), 1.0},
      {cell_t(-1, 0), 1.0},
      {cell_t(0, -1), 1.0},
      {cell_t(1, 1), M_SQRT2},
      {cell_t(1, -1), M_SQRT2},
      {cell_t(-1, 1), M_SQRT2},
      {cell_t(-1, -1), M_SQRT2},
  }};

  std::vector<Edge> neighbors;
  neighbors.reserve(expansion_dirs.size());

  for (const Edge& expansion_dir : expansion_dirs) {
    const cell_t dest = cell + expansion_dir.dest;
    if (distances.isCellInGrid(dest.x, dest.y) &&
        distances(dest.x, dest.y) >= params.minDistanceToObstacle) {
      neighbors.emplace_back(dest, expansion_dir.weight);
    }
  }

  return neighbors;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(const std::vector<cell_t>& nodes,
                                                       const ObstacleDistanceGrid& distances) {
  std::vector<mbot_lcm_msgs::pose2D_t> path;
  ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
  // This should turn the node path into a vector of poses (with heading) in the global frame
  // You should prune the path to get a waypoint path suitable for sending to motion controller

  return path;
}

std::vector<cell_t> prune_node_path(const std::vector<cell_t>& nodePath) {
  std::vector<cell_t> new_node_path;
  ////////////////// TODO: Optionally implement a prune_node_path function
  /////////////////////////////
  // This should remove points in the path along the same line

  return new_node_path;
}
