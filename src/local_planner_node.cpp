#include "Planner.h"

int main(int argc, char **argv)
{
  // Node declaration
  ros::init(argc, argv, "local_planner");
  ros::NodeHandle n;

  ROS_INFO("Initializing planner...");
  Planner planner;
  ROS_INFO("Planner initialized!");

  // Subscribers and Publishers
  ROS_INFO("Setting up subscribers and publishers...");
  ros::Subscriber sub1 = n.subscribe("map", 1, &Planner::CallbackCostMap, &planner);
  ros::Subscriber sub2 = n.subscribe("odometry", 1, &Planner::CallbackStart, &planner);
  ros::Subscriber sub3 = n.subscribe("goal", 1, &Planner::CallbackGoal, &planner);
  ros::Publisher pub1 = n.advertise<nav_msgs::Path>("path", 5);
  ROS_INFO("Subs and Pubs set up!");

  // Params
  ROS_INFO("Reading in params...");
  n.param<std::string>("local_planner/fixed_frame", planner.fixedFrame, "world");
  n.param("local_planner/min_obstacle_proximity", planner.minDistanceFromObstacles, (float)0.2);
  n.param("local_planner/resolution", planner.voxelSize, (float)0.05);

  float update_rate;
  n.param("local_planner/update_rate", update_rate, (float)5.0);
  ros::Rate r(update_rate); // 5 Hz
  ROS_INFO("Params read!");

  // Main Loop
  ROS_INFO("Starting main loop...");
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    Voxel start = planner.costMap.Query(planner.start.x, planner.start.y, planner.start.z);
    Voxel goal = planner.costMap.Query(planner.goal.x, planner.goal.y, planner.goal.z);
    if (planner.firstGoal && planner.firstStart) {
      ROS_INFO("Requesting path from (%0.2f, %0.2f, %0.2f, %0.1f) to (%0.2f, %0.2f, %0.2f, %0.1f)",
                planner.start.x, planner.start.y, planner.start.z, start.distanceToObstacle, 
                planner.goal.x, planner.goal.y, planner.goal.z, goal.distanceToObstacle);
      if ((start.distanceToObstacle >= planner.minDistanceFromObstacles) && (start.distanceToObstacle >= planner.minDistanceFromObstacles)) {
        int result = planner.PlanPath(planner.start, planner.goal);
        if (result == 1) ROS_INFO("No feasible path.");
        if (result == 0) ROS_INFO("Path planned!");
      }
      else {
        ROS_INFO("Start state or goal state is not feasible.");
      }
    }
    else {
      if (!planner.firstStart) ROS_INFO("Waiting for start state...");
      if (!planner.firstGoal) ROS_INFO("Waiting for goal state...");
    }
    if (planner.pathMsg.poses.size() > 0) pub1.publish(planner.pathMsg);
  }
}