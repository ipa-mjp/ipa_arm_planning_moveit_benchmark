
#include <ipa_arm_planning_moveit_benchmark/BenchmarkExecutor.h>
#include <moveit/version.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/regex.hpp>
#include <boost/progress.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <unistd.h>

using namespace moveit_ros_benchmarks;

static std::string getHostname()
{
  static const int BUF_SIZE = 1024;
  char buffer[BUF_SIZE];
  int err = gethostname(buffer, sizeof(buffer));
  if (err != 0)
    return std::string();
  else
  {
    buffer[BUF_SIZE - 1] = '\0';
    return std::string(buffer);
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

//initilaize planning plugin class loader
BenchmarkExecutor::BenchmarkExecutor(const std::string& robot_description_param)
{
	psm_ = NULL;
	psws_ = NULL;
	rs_ = NULL;
	cs_ = NULL;
	tcs_ = NULL;

	psm_ = new planning_scene_monitor::PlanningSceneMonitor(robot_description_param);
	planning_scene_ = psm_->getPlanningScene();

	// Initialize the class loader for planner plugin
	try
	{
		planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex)
	{
	  ROS_FATAL_STREAM("BenchmarkExecutor --> Exception while creating planning plugin loader " << ex.what());
	}
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Clear all allocated memory
BenchmarkExecutor::~BenchmarkExecutor()
{
	if (pss_)
		delete pss_;
	if (psws_)
		delete psws_;
	if (rs_)
		delete rs_;
	if (cs_)
		delete cs_;
	if (tcs_)
		delete tcs_;
	delete psm_;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * initialize benchmarkExecutor class by loading planner plugin, load planner plugins for running benchmark and creat unique instance of each planner plugin
 * @param plugin_classes = avaliable plugins on parameter server which is set on yaml file
**/
void BenchmarkExecutor::initializeBenchmarkExecutor(const std::vector<std::string>& plugin_classes)
{

	planner_interfaces_.clear();

	group.reset(new moveit::planning_interface::MoveGroup("arm"));

	// Load the planning plugins
	const std::vector<std::string>& classes = planner_plugin_loader_->getDeclaredClasses();

	for (std::size_t i = 0; i < plugin_classes.size(); ++i)
	{
		std::vector<std::string>::const_iterator it = std::find(classes.begin(), classes.end(), plugin_classes[i]);
		if (it == classes.end())
		{
			ROS_ERROR("initializeBenchmarkExecutor --> Failed to find plugin_class %s", plugin_classes[i].c_str());
			return;
		}
		try
		{
			planning_interface::PlannerManagerPtr p = planner_plugin_loader_->createUniqueInstance(plugin_classes[i]);

			if	(!p->initialize(planning_scene_->getRobotModel(), ros::this_node::getName()))
				ROS_FATAL_STREAM("initializeBenchmarkExecutor --> Could not initialize planner interfaces");

			planner_interfaces_[plugin_classes[i]] = p;
		}

		catch (pluginlib::PluginlibException& ex)
		{
			ROS_ERROR_STREAM("initializeBenchmarkExecutor --> Exception while loading planner '" << plugin_classes[i] << "': " << ex.what());
		}
	}

	// error check
	if (planner_interfaces_.empty())
		ROS_ERROR("initializeBenchmarkExecutor --> No planning plugins have been loaded. Nothing to do for the benchmarking service.");
	else
	{
		std::stringstream ss;
		for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator it = planner_interfaces_.begin(); it != planner_interfaces_.end(); ++it)
			ss << it->first << " ";
		ROS_WARN("initializeBenchmarkExecutor --> Available planner instances: %s", ss.str().c_str());
	  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
void BenchmarkExecutor::clear()
{
  if (pss_)
  {
    delete pss_;
    pss_ = NULL;
  }
  if (psws_)
  {
    delete psws_;
    psws_ = NULL;
  }
  if (rs_)
  {
    delete rs_;
    rs_ = NULL;
  }
  if (cs_)
  {
    delete cs_;
    cs_ = NULL;
  }
  if (tcs_)
  {
    delete tcs_;
    tcs_ = NULL;
  }

  benchmark_data_.clear();
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
/**
 * Plan using planning scene context of every available plugins with one queries.
 * @param request = motion plan request / set gola state
 * @param planner = iterate over all plugins
 * @param runs = no. of time want to run each plugins
 */
void BenchmarkExecutor::executeBenchmark(moveit_msgs::MotionPlanRequest request, const std::map<std::string, std::vector<std::string>>& planners, int runs)
{
//TODO: ompl consider by default planner not load define planner checkit out and correct it
	benchmark_data_.clear();

	for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end(); ++it)
	{
		for (std::vector<std::string>::const_iterator planner_it = it->second.begin(); planner_it!= it->second.end(); ++planner_it)
		{
			PlannerBenchmarkData planner_data(runs);

			planning_interface::PlanningContextPtr contex = planner_interfaces_[it->first]->getPlanningContext(planning_scene_, request);

			for (int j = 0; j < runs; ++j)
			{
				ROS_WARN_STREAM(j);
				planning_interface::MotionPlanDetailedResponse mp_res;
//				planning_interface::MotionPlanResponse res;
				ros::WallTime start = ros::WallTime::now();
				bool solved = contex->solve(mp_res);

				if (mp_res.error_code_.val != mp_res.error_code_.SUCCESS)
				{
					ROS_ERROR("executeBenchmark --> Could not compute plan successfully");
					solved = false;
				}
				double total_time = (ros::WallTime::now() - start).toSec();

/*  				bool solved1 = contex->solve(res);

				moveit_msgs::DisplayTrajectory display_trajectory;

				// Visualize the trajectory
				ROS_INFO("Visualizing the trajectory");
				moveit_msgs::MotionPlanResponse response;
				res.getMessage(response);

				display_trajectory.trajectory_start = response.trajectory_start;
				display_trajectory.trajectory.push_back(response.trajectory);
				options_.display_publisher_.publish(display_trajectory);
*/
				//collect data
				start = ros::WallTime::now();

				collectMetrics(planner_data[j], mp_res, solved, total_time);
				double metrics_time = (ros::WallTime::now() - start).toSec();
				ROS_DEBUG("Spent %lf seconds collecting metrics", metrics_time);

			}

			benchmark_data_.push_back(planner_data);
		}
	}
}


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool BenchmarkExecutor::runBenchmarks(const BenchmarkOptions& opts)
{
	if (planner_interfaces_.size() == 0)
	{
		ROS_ERROR("runBenchmarks --> No planning interfaces configured.  Did you call BenchmarkExecutor::initializeBenchmarkExecutor?");
		return false;
	}

	moveit_msgs::PlanningScene scene_msg;
	std::vector<BenchmarkRequest> queries;

	if (initializeBenchmarks(opts, scene_msg, queries))
	{
		if (!queriesAndPlannersCompatible(queries))
		{
			ROS_ERROR("runBenchmarks --> Interface cannot service the benchmark request");
			return false;
		}

		for (std::size_t i = 0; i < queries.size(); ++i)
		{
			// Clear all geometry from the scene
			planning_scene_->getWorldNonConst()->clearObjects();
			planning_scene_->getCurrentStateNonConst().clearAttachedBodies();
			planning_scene_->getCurrentStateNonConst().setToDefaultValues();

			planning_scene_->processPlanningSceneWorldMsg(scene_msg.world);

			ROS_INFO("Benchmarking query '%s' (%lu of %lu)", queries[i].name.c_str(), i + 1, queries.size());
			ros::WallTime start_time = ros::WallTime::now();

			executeBenchmark(queries[i].request, opts.getPlannerConfigurations(), opts.getNumRuns());
			double duration = (ros::WallTime::now() - start_time).toSec();
			writeOutput(queries[i], boost::posix_time::to_iso_extended_string(start_time.toBoost()), duration);
		}
		return true;
	}
	return false;
}

/**
 * Make sure queries request is compatible to specified planner plugin / check all parameter required for plan is specified or not
 * @param request = motion planning request (moveit_msgs::MotionPlanRequest)
 * @return bool = true if compatible or can able to represent planning request else false
 */
bool BenchmarkExecutor::queriesAndPlannersCompatible(const std::vector<BenchmarkRequest>& requests)
{
  // Make sure that the planner interfaces can service the desired queries
  for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator it = planner_interfaces_.begin(); it != planner_interfaces_.end(); ++it)
  {
    for (std::size_t i = 0; i < requests.size(); ++i)
    {
      //Determine whether it->second plugin instance is able to represent this planning request (requests[i].request).
      if (!it->second->canServiceRequest(requests[i].request))
      {
        ROS_ERROR("queriesAndPlannersCompatible --> Interface '%s' cannot service the benchmark request '%s'", it->first.c_str(), requests[i].name.c_str());
        return false;
      }
    }
  }

  return true;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * Populate all data members of this class and use it for running benchmark
 * @param opts = get parameter from parameter server which is set in yaml file
 * @param scene_msgs = use for set constraints in worlds and robot
 * @param request = populate motion planning request and name of it
 * @return bool = true for successfully initilaization else false
 */
bool BenchmarkExecutor::initializeBenchmarks(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg, std::vector<BenchmarkRequest>& requests)
{
	if (!plannerConfigurationsExist( opts.getPlannerConfigurations(), opts.getGroupName()))
		return false;

	// connect storage data member to storage.
	try
	{
		warehouse_ros::DatabaseConnection::Ptr connect = dbloader.loadDatabase();
		connect->setParams(opts.getHostName(), opts.getPort(), 20);
		if (connect->connect())
		{
			pss_ = new moveit_warehouse::PlanningSceneStorage(connect);
			psws_ = new moveit_warehouse::PlanningSceneWorldStorage(connect);
			rs_ = new moveit_warehouse::RobotStateStorage(connect);
		    cs_ = new moveit_warehouse::ConstraintsStorage(connect);
		    tcs_ = new moveit_warehouse::TrajectoryConstraintsStorage(connect);
		}
		else
		{
			ROS_ERROR("initializeBenchmarks --> Failed to connect to DB");
			return false;
		}
	}
	catch (std::exception& e)
	{
		ROS_ERROR("initializeBenchmarks --> Failed to initialize benchmark server: '%s'", e.what());
		return false;
	}

	std::vector<StartState> start_states;
	std::vector<BenchmarkRequest> queries;	//goal state
	std::vector<PathConstraints> path_constraints;

	bool ok =	loadPlanningScene(opts.getSceneName(), scene_msg) && loadStates(opts.getStartStateRegex(), start_states) &&
				loadQueries(opts.getQueryRegex(), opts.getSceneName(), queries) && loadPathConstraints( path_constraints);

	if (! ok)
	{
	    ROS_ERROR("initializeBenchmarks --> Failed to load benchmark stuff");
	    return false;
	}

	else
		ROS_INFO("Benchmark loaded %lu starts , %lu path constraints, and %lu queries", start_states.size(), path_constraints.size(), queries.size());

//	ROS_WARN_STREAM("group_name: "<<queries[0].name);
//	ROS_INFO_STREAM("queries: "<<queries[0].request.start_state.joint_state);
//	ROS_WARN_STREAM("start_state: "<<start_states[0].state);

	for (std::vector<BenchmarkRequest>::const_iterator it = queries.begin(); it!= queries.end(); ++it)
	{
		BenchmarkRequest brequest;
		brequest.name = it->name;
		brequest.request = it->request;
		brequest.request.group_name = opts.getGroupName();
		brequest.request.allowed_planning_time = opts.getTimeout();
		brequest.request.num_planning_attempts = 1;
		brequest.request.path_constraints = path_constraints[0].constraints[0];
		brequest.request.start_state = start_states[0].state;
		requests.push_back(brequest);
/*
		std::vector<BenchmarkRequest> request_combs;
		std::vector<PathConstraints> no_path_constraints;
		createRequestCombinations(brequest, start_states, no_path_constraints, request_combs);
		requests.insert(requests.end(), request_combs.begin(), request_combs.end());*/
	}
	options_ = opts;
	return true;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Create request combination of start state and path constraints
void BenchmarkExecutor::createRequestCombinations(const BenchmarkRequest& brequest, const std::vector<StartState>& start_states,
												  const std::vector<PathConstraints>& path_constraints, std::vector<BenchmarkRequest>& request_combs )
{
	if (start_states.empty())
	{
		for (std::vector<PathConstraints>::const_iterator it = path_constraints.begin(); it != path_constraints.end(); ++it)
		{
			BenchmarkRequest new_request = brequest;
			new_request.request.path_constraints = it->constraints[0];
			new_request.name = " ";
			request_combs.push_back(new_request);
		}
		if (path_constraints.empty())
			request_combs.push_back(brequest);
	}

	else
	{
		for (std::vector<StartState>::const_iterator it_state = start_states.begin(); it_state!= start_states.end(); ++it_state)
		{
			BenchmarkRequest new_brequest = brequest;
			new_brequest.request.start_state = it_state->state;
			new_brequest.name = " ";

			for (std::vector<PathConstraints>::const_iterator it = path_constraints.begin(); it != path_constraints.end(); ++it)
			{
				new_brequest.request.path_constraints = it->constraints[0];
				request_combs.push_back(new_brequest);
			}
			if (path_constraints.empty())
				request_combs.push_back(new_brequest);

		}
	}
}



//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * Populate planning scene either whole planning scene or world planning scene from storage
 * @param scene_name = name of scene which is define in yaml file
 * @param & @return scene_msgs = set constraints in worlds and/or robot and also fill it
 * @return bool = true for successfully loading else false
 */
bool BenchmarkExecutor::loadPlanningScene(const std::string& scene_name, moveit_msgs::PlanningScene& scene_msg)
{
	bool ok = false;
	try
	{
		if (pss_->hasPlanningScene(scene_name))	//whole planning
		{
			moveit_warehouse::PlanningSceneWithMetadata pswm;
			ok = pss_->getPlanningScene(pswm, scene_name);
			scene_msg = static_cast< moveit_msgs::PlanningScene>(*pswm);

			if (!ok)
				ROS_ERROR("loadPlanningScene --> Failed to load planning scene '%s'", scene_name.c_str());
			ROS_INFO("loadPlanningScene --> Loaded planning scene successfully");
		}
		else if(psws_->hasPlanningSceneWorld(scene_name))	// Just the world (no robot)
		{
			moveit_warehouse::PlanningSceneWorldWithMetadata pswwm;
			ok = psws_->getPlanningSceneWorld(pswwm, scene_name);
			scene_msg.world = static_cast< moveit_msgs::PlanningSceneWorld>(*pswwm);
			scene_msg.robot_model_name = "NO ROBOT INFORMATION. ONLY WORLD GEOMETRY";  // this will be fixed when running benchmark
			if (!ok)
				ROS_ERROR("loadPlanningScene --> Failed to load planning scene '%s'", scene_name.c_str());
			ROS_INFO("loadPlanningScene --> Loaded planning scene successfully");
		}
		else
		{
			ROS_ERROR("loadPlanningScene --> Failed to find planning scene '%s'", scene_name.c_str());
		}
	}
	catch (std::exception& ex)
	  {
	    ROS_ERROR("loadPlanningScene --> Error loading planning scene: %s", ex.what());
	  }

	return ok;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * Populate all start states from storage
 * @param regex = name of start state which is define in yaml file
 * @param & @return start_states = store/fill start state of robot from storage
 * @return bool = true if start state available or not
 */
bool BenchmarkExecutor::loadStates(const std::string& regex, std::vector<StartState>& start_states)
{
	// check start state is empty or not
	if (regex.size())
	{
		//typedef basic_regex<char>      regex;
		boost::regex start_regex(regex);
		std::vector<std::string> state_names;
		rs_->getKnownRobotStates(state_names);

		for(std::size_t i = 0; i < state_names.size(); ++i)
		{
			moveit_warehouse::RobotStateWithMetadata robot_state;
			try
			{
				if (rs_->getRobotState(robot_state, state_names[i]))
				{
					StartState start_state;
					start_state.name = state_names[i];
					start_state.state = moveit_msgs::RobotState(*robot_state);
					start_states.push_back(start_state);
				}
			}
			catch (std::exception& ex)
			{
				ROS_ERROR("loadStates --> Runtime error when loading state '%s': %s", state_names[i].c_str(), ex.what());
				continue;
			}
		}

		if (start_states.empty())
			ROS_WARN("loadStates --> No stored states matched the provided start state regex: '%s'", regex.c_str());
		else
			ROS_INFO("loadStates --> Loaded states successfully");
	}

	return true;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * Populate all queries of single scene from storage
 * @param regex = name of queries which is define in yaml file
 * @param scene_name = queries of particular scene
 * @param & @return queries = store/fill queries or goal state of robot from storage
 * @return bool = true if start state available or not
 */
bool BenchmarkExecutor::loadQueries(const std::string& regex, const std::string& scene_name, std::vector<BenchmarkRequest>& queries)
{
	// check if queries/goal are there or not
	if (regex.empty())
	{
		ROS_WARN("loadQueries --> No queries are available. Did you set Queries?");
		return true;
	}
	// Make sure has queries
	std::vector<std::string> query_names;
	try
	{
		pss_->getPlanningQueriesNames(regex, query_names, scene_name);
	}
	catch (std::exception& ex)
	{
		ROS_ERROR("loadQueries --> Error loading motion planning queries: %s", ex.what());
		return false;
	}


	if (query_names.empty())
	{
	    ROS_ERROR("loadQueries --> Scene '%s' has no associated queries", scene_name.c_str());
	    return false;
	}

	for (std::vector<std::string>::const_iterator it = query_names.begin(); it != query_names.end(); ++it)
	{
		moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
	    try
	    {
	      pss_->getPlanningQuery(planning_query, scene_name, *it);
	    }
	    catch (std::exception& ex)
	    {
	      ROS_ERROR("loadQueries --> Error loading motion planning query '%s': %s", it->c_str(), ex.what());
	      continue;
	    }
	    BenchmarkRequest query;
	    query.name = *it;
	    query.request = static_cast<moveit_msgs::MotionPlanRequest> (*planning_query);
	    queries.push_back(query);
	}

	if (queries.empty())
		ROS_WARN("loadQueries --> No stored queries matched the provided queries regex: '%s'", regex.c_str());
	else
		ROS_INFO("loadQueries --> Loaded queries successfully");

	return true;
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool BenchmarkExecutor::loadPathConstraints(std::vector<PathConstraints>& constraints)
{
	PathConstraints constraint;
	constraint.constraints.push_back(options_.getConstraints());
	constraint.name = " ";

	constraints.push_back(constraint);
	if (constraints.empty())
		ROS_WARN("loadPathConstraints --> No path constraints found");
	else
		ROS_INFO("loadPathConstraints --> Loaded path constraints successfully");

	return true;
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * check correct planner plugin and planner available
 * @param planners = name of plugin and no. of planners of that plugin
 * @param group_name = name of group ex. manipulator, arm
 * @return bool = if configuration exist return true else false
 */
bool BenchmarkExecutor::plannerConfigurationsExist(const std::map<std::string, std::vector<std::string>>& planners, const std::string& group_name)
{
	// Make sure planner plugins exist
	for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end(); ++it)
	  {
	    bool plugin_exists = false;
	    for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator planner_it = planner_interfaces_.begin(); planner_it != planner_interfaces_.end() && !plugin_exists; ++planner_it)
	    {
	      ROS_WARN_STREAM(planner_it->first.c_str());
	      plugin_exists = planner_it->first == it->first;
	    }

	    if (!plugin_exists)
	    {
	      ROS_ERROR("plannerConfigurationsExist --> Planning plugin '%s' does NOT exist", it->first.c_str());
	      return false;
	    }
	  }

	  // Make sure planning algorithms exist within those plugins
	  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end(); ++it)
	  {
	    planning_interface::PlannerManagerPtr pm = planner_interfaces_[it->first];
	    const planning_interface::PlannerConfigurationMap& config_map = pm->getPlannerConfigurations();

	    for (std::size_t i = 0; i < it->second.size(); ++i)
	    {
	      bool planner_exists = false;
	      for (planning_interface::PlannerConfigurationMap::const_iterator map_it = config_map.begin(); map_it != config_map.end() && !planner_exists; ++map_it)
	      {
	        planner_exists = (map_it->second.group == group_name);
	        if (!planner_exists)
	        {
	        ROS_ERROR("plannerConfigurationsExist --> Planner '%s' does NOT exist for group '%s' in pipeline '%s'", it->second[i].c_str(), group_name.c_str(), it->first.c_str());
	        std::cout << "There are " << config_map.size() << " planner entries: " << std::endl;
	        std::cout <<"plannerConfigurationsExist --> Available planners: "<<std::endl;
	        for (planning_interface::PlannerConfigurationMap::const_iterator map_it = config_map.begin(); map_it != config_map.end() && !planner_exists; ++map_it)
	          std::cout << map_it->second.name << std::endl;
	        return false;
	        }
	      }
	    }
	  }

return true;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void BenchmarkExecutor::collectMetrics(PlannerRunData& metrics, const planning_interface::MotionPlanDetailedResponse& mp_res, bool solved, double total_time)
{
  metrics["time REAL"] = boost::lexical_cast<std::string>(total_time);
  metrics["solved BOOLEAN"] = boost::lexical_cast<std::string>(solved);

  if (solved)
  {
    // Analyzing the trajectory(ies) geometrically
    double L = 0.0;           // trajectory length
    double clearance = 0.0;   // trajectory clearance (average)
    double smoothness = 0.0;  // trajectory smoothness (average)
    bool correct = true;      // entire trajectory collision free and in bounds

    double process_time = 1.0;

    //iterate over no of trajectory, it should be 1.
    for (std::size_t j = 0; j < mp_res.trajectory_.size(); ++j)
    {
     //ROS_WARN_STREAM("size: "<<mp_res.trajectory_.size());

      correct = true;
      L = 0.0;
      clearance = 0.0;
      smoothness = 0.0;
      const robot_trajectory::RobotTrajectory& p = *mp_res.trajectory_[j];

      //ROS_WARN_STREAM("wayPoint: "<< p.getWayPointCount());

      // compute path length
      for (std::size_t k = 1; k < p.getWayPointCount(); ++k)
        L += p.getWayPoint(k - 1).distance(p.getWayPoint(k));

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
    	    	  double angle = (boost::math::constants::pi<double>() - acos(acosValue));

    	    	  // and we normalize by the length of the segments
    	    	  double u = 2.0 * angle;  /// (a + b);
    	    	  smoothness += u * u;
    	      }
    	      a = b;
    	  }
    	  smoothness /= double(p.getWayPointCount());
      }

      // Store all data into metrics
      metrics["path_plan_length REAL"] = boost::lexical_cast<std::string>(L);
      metrics["path_plan_smoothness REAL"] = boost::lexical_cast<std::string>(smoothness);
      metrics["path_plan_time REAL"] = boost::lexical_cast<std::string>(mp_res.processing_time_[j]);
      process_time -= mp_res.processing_time_[j];
    }

    if (process_time <= 0.0)	process_time = 0.0;
    metrics["process_time REAL"] = boost::lexical_cast<std::string>(process_time);

  }
}

void BenchmarkExecutor::writeOutput(const BenchmarkRequest& brequest, const std::string& start_time, double benchmark_duration)
{
	const std::map<std::string, std::vector<std::string>>& planners = options_.getPlannerConfigurations();
	size_t num_planners = 0;
	//store total no planner use in benchmark
	for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end(); ++it)
		num_planners += it->second.size();

	std::string hostname = getHostname();
	if (hostname.empty())
		hostname = "UNKNOWN";

	std::string filename = options_.getOutputDirectory();
	  if (filename.size() && filename[filename.size() - 1] != '/')
	    filename.append("/");

	  // Ensure directories exist
	  boost::filesystem::create_directories(filename);

	  filename += (options_.getBenchmarkName().empty() ? "" : options_.getBenchmarkName() + "_") + brequest.name + "_" +
	              getHostname() + "_" + start_time + ".log";
	  std::ofstream out(filename.c_str());
	  if (!out)
	  {
	    ROS_ERROR("Failed to open '%s' for benchmark output", filename.c_str());
	    return;
	  }

	  out << "MoveIt! version " << MOVEIT_VERSION << std::endl;
	  out << "Experiment " << brequest.name << std::endl;
	  out << "Running on " << hostname << std::endl;
	  out << "Starting at " << start_time << std::endl;

	  // Experiment setup
	  moveit_msgs::PlanningScene scene_msg;
	  planning_scene_->getPlanningSceneMsg(scene_msg);
	  out << "<<<|" << std::endl;
	  out << "Motion plan request:" << std::endl << brequest.request << std::endl;
	  out << "Planning scene: " << std::endl << scene_msg << std::endl << "|>>>" << std::endl;

	  // Not writing optional cpu information

	  // The real random seed is unknown.  Writing a fake value
	  out << "0 is the random seed" << std::endl;
	  out << brequest.request.allowed_planning_time << " seconds per run" << std::endl;
	  // There is no memory cap
	  out << "-1 MB per run" << std::endl;
	  out << options_.getNumRuns() << " runs per planner" << std::endl;
	  out << benchmark_duration << " seconds spent to collect the data" << std::endl;

	  // No enum types
	  out << "0 enum types" << std::endl;

	  out << num_planners << " planners" << std::endl;

	  size_t run_id = 0;
	  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end(); ++it)
	  {
	    for (std::size_t i = 0; i < it->second.size(); ++i, ++run_id)
	    {
	      // Write the name of the planner.
	      out << it->second[i] << std::endl;
	      // in general, we could have properties specific for a planner;
	      // right now, we do not include such properties
	      out << "0 common properties" << std::endl;

	      // Create a list of the benchmark properties for this planner
	      std::set<std::string> properties_set;
	      for (std::size_t j = 0; j < benchmark_data_[run_id].size(); ++j)  // each run of this planner
	        for (PlannerRunData::const_iterator pit = benchmark_data_[run_id][j].begin(); pit != benchmark_data_[run_id][j].end(); ++pit)  // each benchmark property of the given run
	          properties_set.insert(pit->first);

	      // Writing property list
	      out << properties_set.size() << " properties for each run" << std::endl;
	      for (std::set<std::string>::const_iterator pit = properties_set.begin(); pit != properties_set.end(); ++pit)
	        out << *pit << std::endl;

	      // Number of runs
	      out << benchmark_data_[run_id].size() << " runs" << std::endl;

	      // And the benchmark properties
	      for (std::size_t j = 0; j < benchmark_data_[run_id].size(); ++j)  // each run of this planner
	      {
	        // Write out properties in the order we listed them above
	        for (std::set<std::string>::const_iterator pit = properties_set.begin(); pit != properties_set.end(); ++pit)
	        {
	          // Make sure this run has this property
	          PlannerRunData::const_iterator runit = benchmark_data_[run_id][j].find(*pit);
	          if (runit != benchmark_data_[run_id][j].end())
	            out << runit->second;
	          out << "; ";
	        }
	        out << std::endl;  // end of the run
	      }
	      out << "." << std::endl;  // end the planner
	    }
	  }

	  out.close();
	  ROS_INFO("Benchmark results saved to '%s'", filename.c_str());
}
