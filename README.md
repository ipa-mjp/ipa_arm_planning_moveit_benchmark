# ipa_arm_planning_moveit_benchmark

Note: Branch: ipa_moveit_config -- origin/indigo_dev, ipa_arm_planning_moveit_benchmark -- benchmark

1. Startup simulation: roslaunch cob_bringup_sim robot.launch robot:=raw3-1 robot_env:=empty
2. Startup benchmark : roslaunch ipa_arm_planning_moveit_benchmark benchmark.launch robot:=raw3-1 

  2.1. Within the Motion Planning RViz plugin, connect to the database by pressing the Connect    button in the Context tab.

  2.2 Save a scene on the Stored Scenes tab and name it ex.Home by double clicking the scene in the list.
 
  2.3 Move the start and goal states of the robot by using the interactive markers. 
	or
      Within the Motion Planning RViz plugin, Update Query(Start State & Goal State) in the Planning tab.

  2.4 Save an associated query for the Home scene and name the query Pick1.
  
  2.5 Also save a start state for the robot on the Stored States tab and name it Start1. 

3. The config file in our case ipa_arm_planning_moveit_benchmark)/benchmark_config/demo1.yaml refers to the scenes, queries and start states used for benchmarking. Modify them appropriately. Set path of output_directory where .log file store.

4. Run the benchmarks: roslaunch ipa_arm_planning_moveit_benchmark benchmark_robot.launch robot:=raw3-1 file_name:=demo1

5. Open terminal. Go to that directory where .log file store. Than type cd ..

6. rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py <path_of_logfile> that will generate .db file.

7. To generate a PDF of plots: http://plannerarena.org/

  7.1 load you file: Goto Change database tab. Browse you .db file.
  7.2 View graph: Goto Overall performance tab. Set Benchmark attribute.	
  

