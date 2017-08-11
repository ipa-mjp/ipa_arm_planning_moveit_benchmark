#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <string.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <fstream>
#include <stomp_moveit/stomp_planner.h>

using namespace ros;
using namespace Eigen;
using namespace moveit::core;
using namespace std;

template<typename T = string>
void printVector(vector<T> vec)
{
	for (typename vector<T>::const_iterator it = vec.begin(); it!= vec.end(); ++it)
		ROS_WARN("value: '%s' ", static_cast<string>(*it).c_str());
}

template<typename T = string>
void printValue(T value)
{
	ROS_WARN("print_value: '%s' ", static_cast<string> (value).c_str());
}


void collectData(const planning_interface::MotionPlanDetailedResponse& res)
{
	double length_traj = 0.0, smoothness;

	for (size_t i = 0; i < res.trajectory_.size(); ++i)
	{
		const robot_trajectory::RobotTrajectory& p = *res.trajectory_[i];

		// compute path length
		for (std::size_t k = 1; k < p.getWayPointCount(); ++k)
			length_traj += p.getWayPoint(k - 1).distance(p.getWayPoint(k));

		//compute smoothness //this is gives interpolation smoothness not path plan smoothness
		if (p.getWayPointCount() > 2)
		{
			double a = p.getWayPoint(0).distance(p.getWayPoint(1));

		    for (std::size_t k = 2; k < p.getWayPointCount(); ++k)
		    {
		    	double b = p.getWayPoint(k-1).distance(p.getWayPoint(k));
		    	double cdist = p.getWayPoint(k-2).distance(p.getWayPoint(k));
		    	double acosValue = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
		    	if (acosValue > -1.0 && acosValue < 1.0)
		    	{
		    	 // the smoothness is actually the outside angle of the one we compute
		    	 double angle = (3.1415 - acos(acosValue));

		    	 // and we normalize by the length of the segments
		    	  double u = 2.0 * angle;  /// (a + b);
		    	  smoothness += u * u;
		    	 }
		    	a = b;
		      }
		   smoothness /= double(p.getWayPointCount());
		  }

		ROS_WARN_STREAM("lenght_traj: " <<length_traj);
		ROS_WARN_STREAM("smoothness: " <<smoothness);
		ROS_WARN_STREAM("time: " <<res.processing_time_[i]);
	}
}



int main (int argc, char *argv[])
{
	ros::init(argc, argv, "gstomp_run_benchmark");
	ros::NodeHandle nh;

	map<string, XmlRpc::XmlRpcValue> config;
	robot_model_loader::RobotModelLoaderPtr loader;
	robot_model::RobotModelPtr robot_model;
	string urdf_file_path, srdf_file_path;

	urdf_file_path = package::getPath("ipa_hardware_config") + "/robots/raw3-1/urdf/raw3-1.urdf";
	srdf_file_path = package::getPath("ipa_moveit_config") + "/robots/raw3-1/moveit/config/raw3-1.srdf";

	ifstream ifs1 (urdf_file_path.c_str());
	string urdf_string((istreambuf_iterator<char>(ifs1)), (istreambuf_iterator<char>()));

	ifstream ifs2 (srdf_file_path.c_str());
	string srdf_string((istreambuf_iterator<char>(ifs2)), (istreambuf_iterator<char>()));

	robot_model_loader::RobotModelLoader::Options opts(urdf_string, srdf_string);
	loader.reset(new robot_model_loader::RobotModelLoader(opts));
	robot_model = loader->getModel();

	if (!robot_model)
	{
	  ROS_ERROR_STREAM("Unable to load robot model from urdf and srdf.");
	  return false;
	}

	stomp_moveit::StompPlanner::getConfigData(nh, config);

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_interface::MotionPlanRequest req;
	//planning_interface::MotionPlanResponse res;
	planning_interface::MotionPlanDetailedResponse res;
	std::string group_name = "arm";

	stomp_moveit::StompPlanner stomp(group_name, config[group_name], robot_model);	//main line for stomp

	req.allowed_planning_time = 10;
	req.group_name =group_name;
	req.num_planning_attempts = 1;

	robot_state::RobotState start = planning_scene->getCurrentState();
	map<string, double> jstart;
	jstart.insert(make_pair("arm_shoulder_pan_joint", -1.02));
	jstart.insert(make_pair("arm_shoulder_lift_joint", -0.40));
	jstart.insert(make_pair("arm_elbow_joint", -2.402));
	jstart.insert(make_pair("arm_wrist_1_joint", -3.48));
	jstart.insert(make_pair("arm_wrist_2_joint", -1.81));
	jstart.insert(make_pair("arm_wrist_3_joint", 0.00));

	start.setVariablePositions(jstart);
	robotStateToRobotStateMsg(start, req.start_state);
	req.start_state.is_diff = true;

	robot_state::RobotState goal = planning_scene->getCurrentState();
	map<string, double> jgoal;
	jgoal.insert(make_pair("arm_shoulder_pan_joint", -1.02));
	jgoal.insert(make_pair("arm_shoulder_lift_joint", -1.32));
	jgoal.insert(make_pair("arm_elbow_joint", -1.74));
	jgoal.insert(make_pair("arm_wrist_1_joint", -3.16));
	jgoal.insert(make_pair("arm_wrist_2_joint", -1.81));
	jgoal.insert(make_pair("arm_wrist_3_joint", 0.00));

	goal.setVariablePositions(jgoal);

	vector<double> dist(7);
	  dist[0] = 0.05;
	  dist[1] = 0.05;
	  dist[2] = 0.05;
	  dist[3] = 0.05;
	  dist[4] = 0.05;
	  dist[5] = 0.05;
	  dist[6] = 0.05;

	  ros::Time t1, t2;
	  t1 = ros::Time::now();
	  const robot_state::JointModelGroup *jmg = goal.getJointModelGroup(group_name);

	  ROS_WARN("Hello");

	  int test_runs = 10;

	  for (int i = 0; i < test_runs; i++)
	    {
	      if (jmg)
	      {
	        robot_state::RobotState new_goal = goal;
	        new_goal.setToRandomPositionsNearBy(jmg, goal, dist);
	        req.goal_constraints.resize(1);
	        req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(new_goal, jmg);
	      }

	      stomp.clear();
	      stomp.setPlanningScene(planning_scene);
	      stomp.setMotionPlanRequest(req);

	      if (!stomp.solve(res))
	            ROS_ERROR_STREAM("STOMP Solver failed:" << res.error_code_);

	      collectData(res);
	    }
	  t2 = ros::Time::now();
	  ROS_ERROR("Average time spent calculating trajectory: %4.10f seconds", (t2-t1).toSec()/test_runs);
	ROS_WARN("done...");
	ros::spin();

return 0;
}


