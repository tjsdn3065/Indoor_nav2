#include "nav2_voronoi_planner/voronoi_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "nlohmann/json.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_voronoi_planner
{

namespace
{

std::string resolvePackageUri(const std::string & uri)
{
  constexpr const char * kPrefix = "package://";
  if (uri.rfind(kPrefix, 0) != 0) {
    return uri;
  }
  const std::string rest = uri.substr(std::strlen(kPrefix));
  const auto slash = rest.find('/');
  if (slash == std::string::npos) {
    throw std::runtime_error("malformed package:// URI: " + uri);
  }
  const std::string pkg = rest.substr(0, slash);
  const std::string sub = rest.substr(slash + 1);
  return ament_index_cpp::get_package_share_directory(pkg) + "/" + sub;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

}  // namespace

void VoronoiPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("VoronoiPlanner: parent lifecycle node is null");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  name_ = std::move(name);
  global_frame_ = costmap_ros_->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".roadmap_path", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".validate_against_costmap", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_nearest_dist", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoothing", rclcpp::ParameterValue(std::string("catmull_rom")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_sample_step", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".shortcut_max_length", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_mode", rclcpp::ParameterValue(std::string("auto")));

  node->get_parameter(name_ + ".roadmap_path", roadmap_path_);
  node->get_parameter(name_ + ".validate_against_costmap", validate_against_costmap_);
  node->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
  node->get_parameter(name_ + ".max_nearest_dist", max_nearest_dist_);
  node->get_parameter(name_ + ".smoothing", smoothing_);
  node->get_parameter(name_ + ".path_sample_step", path_sample_step_);
  node->get_parameter(name_ + ".shortcut_max_length", shortcut_max_length_);
  node->get_parameter(name_ + ".goal_mode", goal_mode_);

  if (smoothing_ != "none" && smoothing_ != "shortcut" && smoothing_ != "catmull_rom") {
    RCLCPP_WARN(logger_,
      "VoronoiPlanner: unknown smoothing='%s', falling back to 'catmull_rom'",
      smoothing_.c_str());
    smoothing_ = "catmull_rom";
  }
  if (goal_mode_ != "exact" && goal_mode_ != "snap" && goal_mode_ != "auto") {
    RCLCPP_WARN(logger_,
      "VoronoiPlanner: unknown goal_mode='%s', falling back to 'auto'",
      goal_mode_.c_str());
    goal_mode_ = "auto";
  }
  if (path_sample_step_ <= 0.0) {
    path_sample_step_ = 0.05;
  }
  if (shortcut_max_length_ <= 0.0) {
    shortcut_max_length_ = 3.0;
  }

  if (roadmap_path_.empty()) {
    RCLCPP_ERROR(logger_, "VoronoiPlanner: '%s.roadmap_path' parameter is required",
      name_.c_str());
    throw std::runtime_error("VoronoiPlanner: roadmap_path is empty");
  }

  const std::string resolved = resolvePackageUri(roadmap_path_);
  RCLCPP_INFO(logger_, "VoronoiPlanner: loading roadmap from %s", resolved.c_str());
  loadRoadmap(resolved);

  if (roadmap_frame_id_ != global_frame_) {
    RCLCPP_WARN(
      logger_,
      "VoronoiPlanner: roadmap frame_id '%s' != costmap global frame '%s'. "
      "Coordinates will not be transformed; ensure they refer to the same frame.",
      roadmap_frame_id_.c_str(), global_frame_.c_str());
  }

  size_t edge_count = 0;
  for (const auto & nbrs : adj_) {
    edge_count += nbrs.size();
  }
  edge_count /= 2;
  RCLCPP_INFO(
    logger_,
    "VoronoiPlanner: roadmap loaded -- %zu nodes, %zu edges (validate_against_costmap=%s, "
    "deferred to first plan request)",
    nodes_.size(), edge_count, validate_against_costmap_ ? "true" : "false");
}

void VoronoiPlanner::cleanup()
{
  nodes_.clear();
  adj_.clear();
  revalidation_done_ = false;
}

void VoronoiPlanner::activate() {}
void VoronoiPlanner::deactivate() {}

void VoronoiPlanner::loadRoadmap(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("VoronoiPlanner: cannot open roadmap file: " + path);
  }
  nlohmann::json j;
  f >> j;

  roadmap_frame_id_ = j.at("meta").at("frame_id").get<std::string>();

  const auto & jnodes = j.at("nodes");
  nodes_.clear();
  nodes_.reserve(jnodes.size());
  for (const auto & jn : jnodes) {
    RoadmapNode n;
    n.x = jn.at("x").get<double>();
    n.y = jn.at("y").get<double>();
    nodes_.push_back(n);
  }

  adj_.assign(nodes_.size(), {});
  for (const auto & je : j.at("edges")) {
    const int u = je.at("u").get<int>();
    const int v = je.at("v").get<int>();
    const double d = je.at("dist").get<double>();
    if (u < 0 || v < 0 || u >= static_cast<int>(nodes_.size()) ||
        v >= static_cast<int>(nodes_.size())) {
      RCLCPP_WARN(logger_, "VoronoiPlanner: skipping edge with out-of-range ids %d <-> %d", u, v);
      continue;
    }
    adj_[u].push_back({v, d});
    adj_[v].push_back({u, d});
  }
}

void VoronoiPlanner::revalidateAgainstCostmap()
{
  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    RCLCPP_WARN(logger_, "VoronoiPlanner: costmap unavailable; skipping edge validation");
    return;
  }

  // Only block on LETHAL_OBSTACLE. The roadmap is already inflated by
  // robot_radius offline, so cells in the inflation zone but not lethal are
  // expected and traversable. Treating NO_INFORMATION as blocked is unsafe
  // because the costmap may not be fully populated when the static layer
  // hasn't replayed the saved map yet.
  auto cellBlocked = [costmap](unsigned int mx, unsigned int my) {
    return costmap->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE;
  };

  auto worldBlocked = [&](double x, double y) {
    unsigned int mx, my;
    if (!costmap->worldToMap(x, y, mx, my)) {
      return true;
    }
    return cellBlocked(mx, my);
  };

  // Collect blocked edges as undirected (min, max) pairs, plus orphan nodes.
  std::vector<bool> orphan(adj_.size(), false);
  std::unordered_set<long long> blocked_pairs;
  auto packPair = [](int a, int b) -> long long {
    if (a > b) std::swap(a, b);
    return (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
  };

  size_t orphan_nodes = 0;
  for (size_t u = 0; u < adj_.size(); ++u) {
    const auto & np = nodes_[u];
    if (worldBlocked(np.x, np.y)) {
      orphan[u] = true;
      ++orphan_nodes;
    }
  }

  size_t blocked_edges = 0;
  for (size_t u = 0; u < adj_.size(); ++u) {
    if (orphan[u]) continue;
    for (const auto & e : adj_[u]) {
      if (e.to < static_cast<int>(u)) continue;  // each pair once
      if (orphan[e.to] || segmentBlocked(nodes_[u].x, nodes_[u].y,
                                          nodes_[e.to].x, nodes_[e.to].y)) {
        blocked_pairs.insert(packPair(static_cast<int>(u), e.to));
        ++blocked_edges;
      }
    }
  }

  for (size_t u = 0; u < adj_.size(); ++u) {
    if (orphan[u]) {
      adj_[u].clear();
      continue;
    }
    auto & nbrs = adj_[u];
    nbrs.erase(
      std::remove_if(nbrs.begin(), nbrs.end(),
        [&](const AdjEdge & e) {
          return orphan[e.to] ||
                 blocked_pairs.count(packPair(static_cast<int>(u), e.to)) > 0;
        }),
      nbrs.end());
  }

  RCLCPP_INFO(
    logger_,
    "VoronoiPlanner: revalidation removed %zu blocked edge(s) and isolated %zu node(s)",
    blocked_edges, orphan_nodes);
}

int VoronoiPlanner::findNearestNode(double x, double y, double max_dist) const
{
  int best_idx = -1;
  double best_d2 = max_dist * max_dist;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (adj_[i].empty()) continue;  // skip orphans
    const double dx = nodes_[i].x - x;
    const double dy = nodes_[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

int VoronoiPlanner::findNearestReachableNode(double x, double y, double max_dist) const
{
  std::vector<std::pair<double, int>> candidates;
  candidates.reserve(nodes_.size());
  const double max_d2 = max_dist * max_dist;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (adj_[i].empty()) continue;  // skip orphans
    const double dx = nodes_[i].x - x;
    const double dy = nodes_[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < max_d2) {
      candidates.emplace_back(d2, static_cast<int>(i));
    }
  }
  std::sort(candidates.begin(), candidates.end());
  for (const auto & [d2, idx] : candidates) {
    if (!segmentBlocked(x, y, nodes_[idx].x, nodes_[idx].y)) {
      return idx;
    }
  }
  return -1;
}

std::vector<int> VoronoiPlanner::dijkstra(
  int start_idx, int goal_idx,
  const std::function<bool()> & cancel_checker) const
{
  const size_t n = nodes_.size();
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> prev(n, -1);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> pq;

  dist[start_idx] = 0.0;
  pq.push({0.0, start_idx});

  size_t pop_count = 0;
  while (!pq.empty()) {
    // Sample the cancel checker once per 1024 pops to keep tight-loop overhead negligible.
    if ((++pop_count & 0x3FFu) == 0u && cancel_checker && cancel_checker()) {
      return {};
    }
    auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) continue;
    if (u == goal_idx) break;
    for (const auto & e : adj_[u]) {
      const double nd = d + e.dist;
      if (nd < dist[e.to]) {
        dist[e.to] = nd;
        prev[e.to] = u;
        pq.push({nd, e.to});
      }
    }
  }

  if (!std::isfinite(dist[goal_idx])) {
    return {};
  }
  std::vector<int> path;
  for (int cur = goal_idx; cur != -1; cur = prev[cur]) {
    path.push_back(cur);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

bool VoronoiPlanner::segmentBlocked(double x0, double y0, double x1, double y1) const
{
  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return true;
  }
  unsigned int mx0, my0, mx1, my1;
  if (!costmap->worldToMap(x0, y0, mx0, my0) ||
      !costmap->worldToMap(x1, y1, mx1, my1)) {
    return true;
  }
  int x = static_cast<int>(mx0);
  int y = static_cast<int>(my0);
  const int xt = static_cast<int>(mx1);
  const int yt = static_cast<int>(my1);
  const int dx = std::abs(xt - x);
  const int dy = std::abs(yt - y);
  const int sx = (x < xt) ? 1 : -1;
  const int sy = (y < yt) ? 1 : -1;
  int err = dx - dy;
  while (true) {
    if (costmap->getCost(static_cast<unsigned int>(x),
                         static_cast<unsigned int>(y)) ==
        nav2_costmap_2d::LETHAL_OBSTACLE) {
      return true;
    }
    if (x == xt && y == yt) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 < dx)  { err += dx; y += sy; }
  }
  return false;
}

std::vector<VoronoiPlanner::Point>
VoronoiPlanner::visibilityShortcut(const std::vector<Point> & in) const
{
  if (in.size() < 3) {
    return in;
  }
  std::vector<Point> out;
  out.reserve(in.size());
  out.push_back(in.front());
  size_t i = 0;
  while (i < in.size() - 1) {
    size_t best = i + 1;
    for (size_t k = i + 2; k < in.size(); ++k) {
      const double dx = in[k].first - in[i].first;
      const double dy = in[k].second - in[i].second;
      if (std::hypot(dx, dy) > shortcut_max_length_) {
        break;
      }
      if (!segmentBlocked(in[i].first, in[i].second, in[k].first, in[k].second)) {
        best = k;
      } else {
        break;
      }
    }
    out.push_back(in[best]);
    i = best;
  }
  return out;
}

std::vector<VoronoiPlanner::Point>
VoronoiPlanner::linearDensify(const std::vector<Point> & in, double step)
{
  if (in.size() < 2) {
    return in;
  }
  std::vector<Point> out;
  out.push_back(in.front());
  for (size_t i = 1; i < in.size(); ++i) {
    const double x0 = in[i - 1].first;
    const double y0 = in[i - 1].second;
    const double x1 = in[i].first;
    const double y1 = in[i].second;
    const double len = std::hypot(x1 - x0, y1 - y0);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int s = 1; s <= n; ++s) {
      const double t = static_cast<double>(s) / n;
      out.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
    }
  }
  return out;
}

std::vector<VoronoiPlanner::Point>
VoronoiPlanner::catmullRomDensify(const std::vector<Point> & in, double step) const
{
  if (in.size() < 2) {
    return in;
  }
  if (in.size() == 2) {
    return linearDensify(in, step);
  }

  // Clamped Catmull-Rom: virtual control points equal to the endpoints.
  auto pt = [&](int idx) -> Point {
    if (idx < 0) return in.front();
    if (idx >= static_cast<int>(in.size())) return in.back();
    return in[idx];
  };

  std::vector<Point> out;
  out.push_back(in.front());
  size_t fallbacks = 0;

  for (size_t i = 0; i + 1 < in.size(); ++i) {
    const Point p0 = pt(static_cast<int>(i) - 1);
    const Point p1 = pt(static_cast<int>(i));
    const Point p2 = pt(static_cast<int>(i) + 1);
    const Point p3 = pt(static_cast<int>(i) + 2);

    const double seg_len = std::hypot(p2.first - p1.first, p2.second - p1.second);
    const int n = std::max(2, static_cast<int>(std::ceil(seg_len / step)));

    std::vector<Point> samples;
    samples.reserve(n);
    bool curve_safe = true;
    for (int s = 1; s <= n; ++s) {
      const double t = static_cast<double>(s) / n;
      const double t2 = t * t;
      const double t3 = t2 * t;
      const double bx = 0.5 * (
        (2.0 * p1.first) +
        (-p0.first + p2.first) * t +
        (2.0 * p0.first - 5.0 * p1.first + 4.0 * p2.first - p3.first) * t2 +
        (-p0.first + 3.0 * p1.first - 3.0 * p2.first + p3.first) * t3);
      const double by = 0.5 * (
        (2.0 * p1.second) +
        (-p0.second + p2.second) * t +
        (2.0 * p0.second - 5.0 * p1.second + 4.0 * p2.second - p3.second) * t2 +
        (-p0.second + 3.0 * p1.second - 3.0 * p2.second + p3.second) * t3);
      samples.emplace_back(bx, by);

      if (auto * costmap = costmap_ros_->getCostmap()) {
        unsigned int mx, my;
        if (!costmap->worldToMap(bx, by, mx, my) ||
            costmap->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          curve_safe = false;
          break;
        }
      }
    }

    if (curve_safe) {
      for (const auto & s : samples) {
        out.push_back(s);
      }
    } else {
      // Replace this segment with a linear densification of the original
      // (already collision-checked) p1->p2 line.
      const std::vector<Point> seg = {p1, p2};
      auto dense = linearDensify(seg, step);
      // Skip duplicate of p1 (already at the end of `out`).
      for (size_t k = 1; k < dense.size(); ++k) {
        out.push_back(dense[k]);
      }
      ++fallbacks;
    }
  }

  if (fallbacks > 0) {
    RCLCPP_DEBUG(logger_,
      "VoronoiPlanner: catmull-rom fallback to linear on %zu segment(s)", fallbacks);
  }
  return out;
}

nav_msgs::msg::Path VoronoiPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  if (start.header.frame_id != global_frame_ ||
      goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      logger_,
      "VoronoiPlanner: start/goal frame must be '%s' (got '%s' / '%s')",
      global_frame_.c_str(),
      start.header.frame_id.c_str(),
      goal.header.frame_id.c_str());
    return path;
  }

  if (nodes_.empty()) {
    RCLCPP_ERROR(logger_, "VoronoiPlanner: empty roadmap");
    return path;
  }

  if (validate_against_costmap_ && !revalidation_done_) {
    revalidateAgainstCostmap();
    revalidation_done_ = true;
    size_t live_nodes = 0;
    size_t live_edges = 0;
    for (const auto & nbrs : adj_) {
      if (!nbrs.empty()) ++live_nodes;
      live_edges += nbrs.size();
    }
    live_edges /= 2;
    RCLCPP_INFO(
      logger_,
      "VoronoiPlanner: revalidation done -- %zu live nodes / %zu live edges remain",
      live_nodes, live_edges);
  }

  const double sx = start.pose.position.x;
  const double sy = start.pose.position.y;
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  // Start side must have a clear line-of-sight to its chosen node, otherwise
  // the off-roadmap leg from the robot to the roadmap is impassable. Try
  // candidates by ascending distance instead of giving up on the single
  // closest one — that single-attempt design caused frequent failures when
  // the robot drifted into the inflation zone or a corner.
  const int start_idx = findNearestReachableNode(sx, sy, max_nearest_dist_);

  // Goal side depends on mode. "snap" intentionally accepts a node without
  // line-of-sight to the actual goal pose (object-style goals whose centroid
  // sits in an obstacle). "exact" requires line-of-sight. "auto" prefers a
  // reachable node and falls back to a snap-style nearest when none is
  // reachable.
  int goal_idx;
  bool goal_link_clear;
  if (goal_mode_ == "snap") {
    goal_idx = findNearestNode(gx, gy, max_nearest_dist_);
    goal_link_clear = false;  // not consulted in snap mode
  } else {
    goal_idx = findNearestReachableNode(gx, gy, max_nearest_dist_);
    goal_link_clear = (goal_idx >= 0);
    if (goal_idx < 0 && goal_mode_ == "auto") {
      goal_idx = findNearestNode(gx, gy, max_nearest_dist_);
    }
    // exact mode keeps goal_idx == -1 → aborts below with a clear log line.
  }

  if (start_idx < 0 || goal_idx < 0) {
    // Probe so the failure tells the user whether the closest live node was
    // out of range or simply blocked from line-of-sight.
    auto probe = [this](double x, double y) {
      double best_live = std::numeric_limits<double>::infinity();
      double best_any = std::numeric_limits<double>::infinity();
      int best_live_idx = -1;
      for (size_t i = 0; i < nodes_.size(); ++i) {
        const double dx = nodes_[i].x - x;
        const double dy = nodes_[i].y - y;
        const double d = std::hypot(dx, dy);
        if (d < best_any) best_any = d;
        if (!adj_[i].empty() && d < best_live) {
          best_live = d;
          best_live_idx = static_cast<int>(i);
        }
      }
      const bool blocked = (best_live_idx >= 0) &&
        segmentBlocked(x, y, nodes_[best_live_idx].x, nodes_[best_live_idx].y);
      return std::make_tuple(best_live, best_any, blocked);
    };
    auto [s_live, s_any, s_blocked] = probe(sx, sy);
    auto [g_live, g_any, g_blocked] = probe(gx, gy);
    RCLCPP_WARN(
      logger_,
      "VoronoiPlanner: could not snap (start_idx=%d goal_idx=%d, max=%.2f, mode=%s). "
      "start nearest live=%.2f%s any=%.2f, goal nearest live=%.2f%s any=%.2f",
      start_idx, goal_idx, max_nearest_dist_, goal_mode_.c_str(),
      s_live, s_blocked ? " (blocked)" : "", s_any,
      g_live, g_blocked ? " (blocked)" : "", g_any);
    return path;
  }

  std::vector<int> idx_path;
  if (start_idx == goal_idx) {
    idx_path = {start_idx};
  } else {
    idx_path = dijkstra(start_idx, goal_idx, cancel_checker);
  }
  if (idx_path.empty()) {
    RCLCPP_WARN(logger_,
      "VoronoiPlanner: no roadmap path from node %d to node %d", start_idx, goal_idx);
    return path;
  }

  // Decide whether the path terminates at the actual goal pose ("exact") or
  // at the nearest reachable roadmap node ("snap"). The reachable goal node
  // (if any) was already line-of-sight-checked in the snap step above.
  bool endpoint_is_goal;
  if (goal_mode_ == "exact") {
    endpoint_is_goal = true;  // findNearestReachableNode guarantees LoS
  } else if (goal_mode_ == "snap") {
    endpoint_is_goal = false;
  } else {  // auto
    endpoint_is_goal = goal_link_clear;
  }

  // Build the raw waypoint sequence.
  std::vector<Point> raw;
  raw.reserve(idx_path.size() + 2);
  raw.emplace_back(sx, sy);
  for (int idx : idx_path) {
    raw.emplace_back(nodes_[idx].x, nodes_[idx].y);
  }
  if (endpoint_is_goal) {
    raw.emplace_back(gx, gy);
  }

  // Drop consecutive duplicates (e.g. start exactly on a node).
  std::vector<Point> dedup;
  dedup.reserve(raw.size());
  for (const auto & p : raw) {
    if (!dedup.empty()) {
      const double dx = p.first - dedup.back().first;
      const double dy = p.second - dedup.back().second;
      if (dx * dx + dy * dy < 1e-8) continue;
    }
    dedup.push_back(p);
  }
  if (dedup.size() < 2) {
    dedup = {{sx, sy}, {gx, gy}};
  }

  // Stage 1: visibility shortcut (cheap, removes zigzag).
  std::vector<Point> waypoints = dedup;
  if (smoothing_ != "none") {
    waypoints = visibilityShortcut(waypoints);
  }

  // Stage 2: smoothing + dense sampling.
  std::vector<Point> dense;
  if (smoothing_ == "catmull_rom") {
    dense = catmullRomDensify(waypoints, path_sample_step_);
  } else if (smoothing_ == "shortcut") {
    dense = linearDensify(waypoints, path_sample_step_);
  } else {  // "none"
    dense = waypoints;
  }

  if (dense.size() < 2) {
    if (endpoint_is_goal) {
      dense = {{sx, sy}, {gx, gy}};
    } else {
      dense = {{sx, sy}, {nodes_[goal_idx].x, nodes_[goal_idx].y}};
    }
  }

  // Final-pose orientation: face the actual goal (the object) when snapping,
  // so the robot stops oriented toward what the user clicked. When the path
  // terminates at the goal pose, honour the user-supplied orientation.
  geometry_msgs::msg::Quaternion final_orientation;
  if (endpoint_is_goal) {
    final_orientation = goal.pose.orientation;
  } else {
    const double yaw = std::atan2(gy - dense.back().second, gx - dense.back().first);
    final_orientation = yawToQuaternion(yaw);
  }

  path.poses.reserve(dense.size());
  for (size_t i = 0; i < dense.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = dense[i].first;
    ps.pose.position.y = dense[i].second;
    ps.pose.position.z = 0.0;
    if (i + 1 < dense.size()) {
      const double dx = dense[i + 1].first - dense[i].first;
      const double dy = dense[i + 1].second - dense[i].second;
      ps.pose.orientation = yawToQuaternion(std::atan2(dy, dx));
    } else {
      ps.pose.orientation = final_orientation;
    }
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace nav2_voronoi_planner

PLUGINLIB_EXPORT_CLASS(nav2_voronoi_planner::VoronoiPlanner, nav2_core::GlobalPlanner)
