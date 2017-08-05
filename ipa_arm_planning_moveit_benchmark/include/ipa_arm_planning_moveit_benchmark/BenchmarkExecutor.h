

#ifndef MOVEIT_ROS_BENCHMARKS_BENCHMARK_EXECUTOR_
#define MOVEIT_ROS_BENCHMARKS_BENCHMARK_EXECUTOR_

#include <ipa_arm_planning_moveit_benchmark/BenchmarkOptions.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/planning_scene_world_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/trajectory_constraints_storage.h>
#include <moveit/planning_interface/planning_interface.h>
#include <warehouse_ros/database_loader.h>
#include <pluginlib/class_loader.h>

#include <map>
#include <vector>
#include <string>
#include <boost/function.hpp>
#include <memory>

namespace moveit_ros_benchmarks
{
/// A class that executes motion plan requests and aggregates data across multiple runs
/// Note: This class operates outside of MoveGroup and does NOT use PlanningRequestAdapters
class BenchmarkExecutor
{
public:

	/// Structure to hold information for a single run of a planner
	typedef std::map<std::string, std::string> PlannerRunData;
	/// Structure to hold information for a single planner's benchmark data.
	typedef std::vector<PlannerRunData> PlannerBenchmarkData;


  BenchmarkExecutor(const std::string& robot_description_param = "robot_description");
  virtual ~BenchmarkExecutor();

  void initializeBenchmarkExecutor(const std::vector<std::string>& plugin_classes);

  virtual bool runBenchmarks(const BenchmarkOptions& opts);

protected:

  struct BenchmarkRequest
  {
	  std::string name;
	  moveit_msgs::MotionPlanRequest request;
  };
  struct StartState
  {
	  std::string name;
	  moveit_msgs::RobotState state;
  };

  virtual bool initializeBenchmarks(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg, std::vector<BenchmarkRequest>& queries);

  virtual void collectMetrics(PlannerRunData& metrics, const planning_interface::MotionPlanDetailedResponse& mp_res, bool solved, double total_time);

  void executeBenchmark(moveit_msgs::MotionPlanRequest request, const std::map<std::string, std::vector<std::string>>& planners, int runs);

  /// Check that the given requests can be run on the set of planner plugins and algorithms
    bool queriesAndPlannersCompatible(const std::vector<BenchmarkRequest>& requests);


  bool plannerConfigurationsExist(const std::map<std::string, std::vector<std::string>>& planners, const std::string& group_name);

  /// Load the planning scene with the given name from the warehouse
  bool loadPlanningScene(const std::string& scene_name, moveit_msgs::PlanningScene& scene_msg);

  /// Load all states matching the given regular expression from the warehouse
  bool loadStates(const std::string& regex, std::vector<StartState>& start_states);

  /// Load all motion plan requests matching the given regular expression from the warehouse
  bool loadQueries(const std::string& regex, const std::string& scene_name, std::vector<BenchmarkRequest>& queries);

  planning_scene_monitor::PlanningSceneMonitor* psm_;
  moveit_warehouse::PlanningSceneStorage* pss_;
  moveit_warehouse::PlanningSceneWorldStorage* psws_;
  moveit_warehouse::RobotStateStorage* rs_;
  moveit_warehouse::ConstraintsStorage* cs_;
  moveit_warehouse::TrajectoryConstraintsStorage* tcs_;

  warehouse_ros::DatabaseLoader dbloader;
  planning_scene::PlanningScenePtr planning_scene_;

  BenchmarkOptions options_;

  std::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
  std::map<std::string, planning_interface::PlannerManagerPtr> planner_interfaces_;

  std::vector<PlannerBenchmarkData> benchmark_data_;

};
}

#endif
