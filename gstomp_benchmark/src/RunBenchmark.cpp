

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

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/pluginlib_exceptions.h>
#include <stomp_moveit/stomp_planner.h>

using namespace ros;
using namespace Eigen;
using namespace moveit::core;
using namespace std;

bool getConfigData(ros::NodeHandle &nh, std::map<std::string, XmlRpc::XmlRpcValue> &config, std::string param)
{
  // Create a stomp planner for each group
  XmlRpc::XmlRpcValue stomp_config;
  if(!nh.getParam(param, stomp_config))
  {
    ROS_ERROR("The 'stomp' configuration parameter was not found");
    return false;
  }

  // each element under 'stomp' should be a group name
  std::string group_name;
  for(XmlRpc::XmlRpcValue::iterator v = stomp_config.begin(); v != stomp_config.end(); v++)
  {
     group_name = static_cast<std::string>(v->second["group_name"]);
     config.insert(std::make_pair(group_name, v->second));
  }

  ROS_WARN("Reading data successfully");
    return true;

}

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


int main (int argc, char *argv[])
{
	ros::init(argc, argv, "gstomp_run_benchmark");
	ros::NodeHandle nh;

	map<string, XmlRpc::XmlRpcValue> config;
	robot_model_loader::RobotModelLoaderPtr loader;
	robot_model::RobotModelPtr robot_model;
	string urdf_file_path, srdf_file_path;

	std::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;

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

	//---------------------------------------- plugin ---------------------------

	std::string planner_plugin_name = "stomp_moveit/StompPlannerManager";
	try
	{
	  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex)
	{
	  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
	}
	try
	{
	  planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
	  if (!planner_instance->initialize(robot_model, nh.getNamespace()))
	    ROS_FATAL_STREAM("Could not initialize planner instance");
	  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
	}
	catch(pluginlib::PluginlibException& ex)
	{
	  const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
	  std::stringstream ss;
	  for (std::size_t i = 0 ; i < classes.size() ; ++i)
	    ss << classes[i] << " ";
	  ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
	                   << "Available plugins: " << ss.str());
	}

	ros::WallDuration sleep_time(5.0);
	sleep_time.sleep();

	//--------------------------------------------------------------------
	//vector<std::string> jnames = robot_model->getJointModelNames();

	//printVector<std::string>(jnames);

	getConfigData(nh, config, "stomp");

	//stomp_moveit::StompPlanner::getConfigData(nh, config);

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;
	std::string group_name = "arm";

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

	start.setVariablePositions(jgoal);

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

	  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

	  ROS_WARN("Hello");

	  int test_runs = 1;

	  for (int i = 0; i < test_runs; i++)
	    {
	      if (jmg)
	      {
	        robot_state::RobotState new_goal = goal;
	        new_goal.setToRandomPositionsNearBy(jmg, goal, dist);
	        req.goal_constraints.resize(1);
	        req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(new_goal, jmg);
	      }
		  ROS_WARN("Hello");
		  context->clear();
	      context->setPlanningScene(planning_scene);
	      context->setMotionPlanRequest(req);
		  ROS_WARN("Hello");
	      if (context->solve(res))
	    	  ROS_ERROR_STREAM("STOMP Solver failed:" << res.error_code_);

	      if(res.error_code_.val != res.error_code_.SUCCESS)
	      {
	      	ROS_ERROR("Could not compute plan successfully");
	      	return 0;
	      }

	    }

	ROS_WARN("done...");
	ros::spin();

return 0;
}


