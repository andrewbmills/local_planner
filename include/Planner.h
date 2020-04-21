#include "Map.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// pcl libraries
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//pcl ROS
#include <pcl_conversions/pcl_conversions.h>

class Planner {
    public:
        Point start;
        Point goal;
        float voxelSize;
        CostMap3D costMap;
        std::string fixedFrame;
        nav_msgs::Path pathMsg;
        bool firstGoal = false;
        bool firstStart = false;
        float minDistanceFromObstacles = 0.0;

        void CallbackCostMap(const sensor_msgs::PointCloud2ConstPtr& msg);
        void CallbackStart(const nav_msgs::Odometry msg);
        void CallbackGoal(const geometry_msgs::PointStamped msg);
        int PlanPath(Point start, Point goal);
        Point FindClosestFeasiblePoint(Point query);
};

void GetPointCloudBounds(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float min[3], float max[3]);

void Planner::CallbackCostMap(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (msg->data.size() == 0) return;

    // Copy cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // Get bounds
    float boundsMin[3], boundsMax[3];
    GetPointCloudBounds(cloud, boundsMin, boundsMax);

    // Copy cloud to costMap data structure
    int size[3];
    for (int i=0; i<3; i++) size[i] = std::roundf((boundsMax[i] - boundsMin[i])/voxelSize) + 1;
    ROS_INFO("Initializing map with min bounds: [%0.1f, %0.1f, %0.1f]", boundsMin[0], boundsMin[1], boundsMin[2]);
    ROS_INFO("Initializing map with max bounds: [%0.1f, %0.1f, %0.1f]", boundsMax[0], boundsMax[1], boundsMax[2]);
    ROS_INFO("Initializing map with sizes: [%d, %d, %d]", size[0], size[1], size[2]);
    costMap = CostMap3D(voxelSize, size, boundsMin);

    for (int i=0; i<cloud->points.size(); i++) {
        pcl::PointXYZI query = cloud->points[i];
        // ROS_INFO("Setting voxel [%0.1f, %0.1f, %0.1f] with intensity %0.1f", query.x, query.y, query.z, query.intensity);
        costMap.SetVoxel(query.x, query.y, query.z, query.intensity);
    }

    return;
}

void Planner::CallbackStart(const nav_msgs::Odometry msg)
{
    if (!firstStart) firstStart = true;
    start.x = msg.pose.pose.position.x;
    start.y = msg.pose.pose.position.y;
    start.z = msg.pose.pose.position.z;
    return;
}

void Planner::CallbackGoal(const geometry_msgs::PointStamped msg)
{
    if (!firstGoal) firstGoal = true;
    ROS_INFO("Goal received!");
    goal.x = msg.point.x;
    goal.y = msg.point.y;
    goal.z = msg.point.z;
    return;
}

int Planner::PlanPath(Point start, Point goal)
{
    // Check if the start and goal points are valid
    if ((!costMap.CheckVoxelPositionInBounds(start)) || (!costMap.CheckVoxelPositionInBounds(goal))) {
        return 1;
    }
    
    float totalCost = 0.0;
    // float startArray[3] = {start.x, start.y, start.z};
    // float goalArray[3] = {goal.x, goal.y, goal.z};
    int startId = costMap.ConvertPositionToIndex(start);
    int goalId = costMap.ConvertPositionToIndex(goal);
    Map astarMap;
    astarMap.costMap = costMap;
    astarMap.minDistanceFromObstacles = minDistanceFromObstacles;
    // int result = astarMap.astar->Solve(reinterpret_cast<void*>(startArray), reinterpret_cast<void*>(goalArray), &(astarMap.path), &totalCost);
    int result = astarMap.astar->Solve(reinterpret_cast<void*>(startId), reinterpret_cast<void*>(goalId), &(astarMap.path), &totalCost);

    nav_msgs::Path newPathMsg;
    for (int i=0; i<astarMap.path.size(); i++) {
        geometry_msgs::PoseStamped newPose;
        // float* pathPositionArray = (float*)astarMap.path[i];
        // Voxel pathVoxel = astarMap.costMap.Query(pathPositionArray[0], pathPositionArray[1], pathPositionArray[2]);
        int voxelId = *((int*)(&(astarMap.path[i])));
        Voxel pathVoxel = costMap.Query(voxelId);
        newPose.pose.position.x = pathVoxel.position.x;
        newPose.pose.position.y = pathVoxel.position.y;
        newPose.pose.position.z = pathVoxel.position.z;
        // newPose.pose.position.x = pathPositionArray[0];
        // newPose.pose.position.y = pathPositionArray[1];
        // newPose.pose.position.z = pathPositionArray[2];
        newPathMsg.poses.push_back(newPose);
    }
    newPathMsg.header.frame_id = fixedFrame;
    newPathMsg.header.stamp = ros::Time();
    pathMsg = newPathMsg;
    return result;
}

void GetPointCloudBounds(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float min[3], float max[3])
{
    Point boundsMin{0.0, 0.0, 0.0};
    Point boundsMax{0.0, 0.0, 0.0};

    // Get the xyz extents of the PCL by running one loop through the data
    for (int i=0; i<cloud->points.size(); i++) {
        pcl::PointXYZI query = cloud->points[i];
        if (i == 0) {
            boundsMin.x = (double)query.x; boundsMin.y = (double)query.y; boundsMin.z = (double)query.z;
            boundsMax.x = (double)query.x; boundsMax.y = (double)query.y; boundsMax.z = (double)query.z;
            continue;
        }
        if (query.x < boundsMin.x) boundsMin.x = (double)query.x;
        if (query.y < boundsMin.y) boundsMin.y = (double)query.y;
        if (query.z < boundsMin.z) boundsMin.z = (double)query.z;
        if (query.x > boundsMax.x) boundsMax.x = (double)query.x;
        if (query.y > boundsMax.y) boundsMax.y = (double)query.y;
        if (query.z > boundsMax.z) boundsMax.z = (double)query.z;
    }

    min[0] = boundsMin.x; min[1] = boundsMin.y; min[2] = boundsMin.z;
    max[0] = boundsMax.x; max[1] = boundsMax.y; max[2] = boundsMax.z;
    return;
}

Point FindClosestFeasiblePoint(Point query)
{
    Point closestPoint;
    
    return closestPoint;
}