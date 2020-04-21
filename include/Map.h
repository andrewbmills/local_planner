#include "micropather.cpp"
#include <cstdint>
#include "CostMap3D.h"
#include <ros/ros.h>

class Map: public Graph {
    public:
        micropather::MicroPather* astar;
        micropather::MPVector<void*> path;
        CostMap3D costMap;
        float minDistanceFromObstacles = 0.0;
        Map() {
            astar = new micropather::MicroPather(this, 1000, 26, false);
        }
        ~Map() {
            delete astar;
        }
        virtual float LeastCostEstimate(void* stateStart, void* stateEnd);
        virtual void AdjacentCost(void* state, MP_VECTOR<micropather::StateCost> *adjacent);
        virtual void PrintStateInfo(void* state);
};

float Map::LeastCostEstimate(void* stateStart, void* stateEnd)
{
    int startId = *((int*)(&stateStart));
    int endId = *((int*)(&stateEnd));
    // float* startArray = (float*)stateStart;
    // float* endArray = (float*)stateEnd;
    // Voxel start = costMap.Query(startArray[0], startArray[1], startArray[2]);
    // Voxel end = costMap.Query(endArray[0], endArray[1], endArray[2]);
    Voxel start = costMap.Query(startId);
    Voxel end = costMap.Query(endId);
    float maxObstacleProximity = std::max(start.distanceToObstacle, end.distanceToObstacle);
    // ROS_INFO("Getting cost2go estimate from (%0.1f, %0.1f, %0.1f) to (%0.1f, %0.1f, %0.1f):",
    //         startArray[0], startArray[1], startArray[2],
    //         endArray[0], endArray[1], endArray[2]);
    // ROS_INFO("Getting cost2go estimate from (%0.1f, %0.1f, %0.1f) to (%0.1f, %0.1f, %0.1f):",
            // start.position.x, start.position.y, start.position.z, end.position.x, end.position.y, end.position.z);
    float dist2 = (end.position.x - start.position.x)*(end.position.x - start.position.x) +
                (end.position.y - start.position.y)*(end.position.y - start.position.y) +
                (end.position.z - start.position.z)*(end.position.z - start.position.z);
    // float dist2 = 0.0;
    // for (int i=0; i<3; i++) dist2 = dist2 + (endArray[i] - startArray[i])*(endArray[i] - startArray[i]);
    // ROS_INFO("Cost2go = %0.2f meters", std::sqrt(dist2));
    return std::sqrt(dist2)/maxObstacleProximity;
    // return std::sqrt(dist2);
}

void Map::AdjacentCost(void* state, MP_VECTOR<micropather::StateCost> *adjacent)
{
    int stateId = *((int*)(&state));
    // float* stateArray = (float*)state;
    // Voxel stateVoxel = costMap.Query(stateArray[0], stateArray[1], stateArray[2]);
    Voxel stateVoxel = costMap.Query(stateId);
    std::vector<Voxel> neighbors;
    std::vector<float> distances;
    std::vector<int> indices;
    // ROS_INFO("Getting neighbors for (%0.1f, %0.1f, %0.1f):", stateVoxel.position.x, stateVoxel.position.y, stateVoxel.position.z);
    costMap.GetVoxelNeighbors3D(stateVoxel.position.x, stateVoxel.position.y, stateVoxel.position.z, neighbors, distances, indices);
    for (int i=0; i<neighbors.size(); i++) {
        // ROS_INFO("Neighbor = (%0.1f, %0.1f, %0.1f) /w obstacleDistance = %0.1f, Distance = %0.1f, and index %d", 
        //         neighbors[i].position.x, neighbors[i].position.y, neighbors[i].position.z,
        //         neighbors[i].distanceToObstacle, distances[i], indices[i]);
        if ((neighbors[i].distanceToObstacle > minDistanceFromObstacles) &&  (neighbors[i].distanceToObstacle > 0.01)) {
            micropather::StateCost stateCost;
            // float neighborArray[3] = {neighbors[i].position.x, neighbors[i].position.y, neighbors[i].position.z};
            stateCost.state = reinterpret_cast<void*>(indices[i]);
            stateCost.cost = distances[i]/neighbors[i].distanceToObstacle;
            // stateCost.cost = distances[i];
            adjacent->push_back(stateCost);
            // ROS_INFO("Neighbor position: (%0.1f, %0.1f, %0.1f) with cost %0.2f", 
            //         neighbors[i].position.x, neighbors[i].position.y, neighbors[i].position.z, stateCost.cost);
        }
    }
    return;
}

void Map::PrintStateInfo(void* state)
{
    int stateId = *((int*)(&state));
    // float* stateArray = (float*)state;
    // Voxel stateVoxel = costMap.Query(stateArray[0], stateArray[1], stateArray[2]);
    Voxel stateVoxel = costMap.Query(stateId);
    std::cout << "Position = [" << stateVoxel.position.x << ", " << stateVoxel.position.y << ", " << stateVoxel.position.z << "]" << std::endl;
    std::cout << "Distance to closest obstacle = " << stateVoxel.distanceToObstacle << std::endl;
    return;
}