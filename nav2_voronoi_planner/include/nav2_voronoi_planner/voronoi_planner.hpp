#ifndef NAV2_VORONOI_PLANNER__VORONOI_PLANNER_HPP_
#define NAV2_VORONOI_PLANNER__VORONOI_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_voronoi_planner
{

struct RoadmapNode
{
  double x;
  double y;
};

struct AdjEdge
{
  int to;
  double dist;
};

class VoronoiPlanner : public nav2_core::GlobalPlanner
{
public:
  VoronoiPlanner() = default;
  ~VoronoiPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  using Point = std::pair<double, double>;

  void loadRoadmap(const std::string & path);
  void revalidateAgainstCostmap();
  int findNearestNode(double x, double y, double max_dist) const;
  // Like findNearestNode but iterates candidates by distance and returns the
  // closest one whose straight-line segment to (x, y) is free of LETHAL cells.
  // Returns -1 if no candidate within max_dist has clear line-of-sight.
  int findNearestReachableNode(double x, double y, double max_dist) const;
  std::vector<int> dijkstra(int start_idx, int goal_idx,
    const std::function<bool()> & cancel_checker) const;

  // True iff the straight world-frame segment (x0,y0)->(x1,y1) crosses any
  // LETHAL_OBSTACLE cell of the current global costmap.
  bool segmentBlocked(double x0, double y0, double x1, double y1) const;

  // Drop intermediate waypoints whose line-of-sight already covers a longer
  // collision-free shortcut. Returns the simplified polyline.
  std::vector<Point> visibilityShortcut(const std::vector<Point> & in) const;

  // Catmull-Rom curve through `in`, sampled at `step` metres. Falls back to
  // a linearly-densified original segment when the smoothed sub-segment hits
  // a lethal cell.
  std::vector<Point> catmullRomDensify(const std::vector<Point> & in,
                                       double step) const;

  static std::vector<Point> linearDensify(const std::vector<Point> & in,
                                          double step);

  // ROS plumbing.
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("VoronoiPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  std::string global_frame_;

  // Parameters.
  std::string roadmap_path_;
  bool validate_against_costmap_{true};
  double goal_tolerance_{0.5};
  double max_nearest_dist_{1.5};
  std::string smoothing_{"catmull_rom"};   // none | shortcut | catmull_rom
  double path_sample_step_{0.05};
  double shortcut_max_length_{3.0};
  std::string goal_mode_{"auto"};          // exact | snap | auto

  // Roadmap.
  std::vector<RoadmapNode> nodes_;
  std::vector<std::vector<AdjEdge>> adj_;
  std::string roadmap_frame_id_;
  bool revalidation_done_{false};
};

}  // namespace nav2_voronoi_planner

#endif  // NAV2_VORONOI_PLANNER__VORONOI_PLANNER_HPP_
