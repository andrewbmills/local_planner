#include <vector>
#include "math.h"
#include <iostream>
#include <ros/ros.h>

struct Point {
    float x, y, z;
};

struct Dimensions {
    int x, y, z;
};

struct Voxel {
    Point position;
    float distanceToObstacle = 0.0; // EDT value
};

class CostMap3D {
    private:
        Dimensions size;
        Point minBounds;
        Point maxBounds;
        std::vector<Voxel> voxels;
        float voxelSize;
        void _SetMaxBounds();
    public:
        CostMap3D();
        CostMap3D(float resolution, int sizeList[3]);
        CostMap3D(float resolution, int sizeList[3], float minBoundList[3]);
        // void ImportData(std::vector<std::vector<Voxel>> data);
        bool CheckVoxelPositionInBounds(Point position);
        int ConvertPositionToIndex(Point position);
        void SetVoxel(float x, float y, float z, float data);
        Voxel Query(int idx);
        Voxel Query(float x, float y, float z);
        void GetVoxelNeighbors3D(float x, float y, float z, std::vector<Voxel> &neighbors, std::vector<float> &distances, std::vector<int> &indices);
};

CostMap3D::CostMap3D()
{
    voxelSize = 1.0;
    size.x = 1;
    size.y = 1;
    size.z = 1;
    minBounds.x = 0.0;
    minBounds.y = 0.0;
    minBounds.z = 0.0;
    _SetMaxBounds();
    voxels.resize(size.x*size.y*size.z);
    return;
}

CostMap3D::CostMap3D(float resolution, int sizeList[3])
{
    voxelSize = resolution;
    size.x = sizeList[0];
    size.y = sizeList[1];
    size.z = sizeList[2];
    minBounds.x = 0.0;
    minBounds.y = 0.0;
    minBounds.z = 0.0;
    _SetMaxBounds();
    voxels.resize(size.x*size.y*size.z);
    return;
}

CostMap3D::CostMap3D(float resolution, int sizeList[3], float minBoundList[3])
{
    voxelSize = resolution;
    size.x = sizeList[0];
    size.y = sizeList[1];
    size.z = sizeList[2];
    minBounds.x = minBoundList[0];
    minBounds.y = minBoundList[1];
    minBounds.z = minBoundList[2];
    _SetMaxBounds();
    voxels.resize(size.x*size.y*size.z);
    return;
}

int CostMap3D::ConvertPositionToIndex(Point position)
{
    if (CheckVoxelPositionInBounds(position)) {
        int id;
        id = std::roundf((position.z - minBounds.z)/voxelSize)*size.x*size.y;
        id += std::roundf((position.y - minBounds.y)/voxelSize)*size.x;
        id += std::roundf((position.x - minBounds.x)/voxelSize);
        return id;
    }
    else {
        return -1;
    }
}

bool CostMap3D::CheckVoxelPositionInBounds(Point position)
{
    // ROS_INFO("Checking if [%0.1f, %0.1f, %0.1f] is within bounds.", position.x, position.y, position.z);
    // ROS_INFO("Lower bounds [%0.1f, %0.1f, %0.1f]", minBounds.x, minBounds.y, minBounds.z);
    // ROS_INFO("Upper bounds [%0.1f, %0.1f, %0.1f]", maxBounds.x, maxBounds.y, maxBounds.z);
    bool inBounds = (position.x >= minBounds.x) &&
    (position.y >= minBounds.y) &&
    (position.z >= minBounds.z) &&
    (position.x <= maxBounds.x) &&
    (position.y <= maxBounds.y) &&
    (position.z <= maxBounds.z);
    return inBounds;
}

void CostMap3D::_SetMaxBounds()
{
    maxBounds.x = minBounds.x + voxelSize*(size.x - 1);
    maxBounds.y = minBounds.y + voxelSize*(size.y - 1);
    maxBounds.z = minBounds.z + voxelSize*(size.z - 1);
    return;
}

void CostMap3D::SetVoxel(float x, float y, float z, float data)
{
    Point query{x, y, z};
    if (CheckVoxelPositionInBounds(query)) {
        int idx = ConvertPositionToIndex(query);
        voxels[idx].position.x = query.x;
        voxels[idx].position.y = query.y;
        voxels[idx].position.z = query.z;
        voxels[idx].distanceToObstacle = data;
        // ROS_INFO("Voxel has id %d and now has data: [%0.1f, %0.1f, %0.1f] with distanceToObstacle %0.1f", idx,
        //         voxels[idx].position.x, voxels[idx].position.y, voxels[idx].position.z, voxels[idx].distanceToObstacle);
    }
    else {
        // std::cout << "Voxel position out of bounds" << std::endl;
    }
    return;
}

Voxel CostMap3D::Query(int idx)
{
    if ((idx >= 0) && (idx < voxels.size())) {
        Voxel outputVoxel;
        outputVoxel.position = voxels[idx].position;
        outputVoxel.distanceToObstacle = voxels[idx].distanceToObstacle;
        return outputVoxel;
    }
    else {
        std::cout << "Voxel id out of bounds" << std::endl;
        Voxel outputVoxel;
        outputVoxel.position.x = 0.0;
        outputVoxel.position.y = 0.0;
        outputVoxel.position.z = 0.0;
        outputVoxel.distanceToObstacle = -1.0;
        return outputVoxel;
    }
}

Voxel CostMap3D::Query(float x, float y, float z)
{
    Point query{x, y, z};
    if (CheckVoxelPositionInBounds(query)) {
        int idx = ConvertPositionToIndex(query);
        Voxel outputVoxel;
        outputVoxel.position = query;
        outputVoxel.distanceToObstacle = voxels[idx].distanceToObstacle;
        return outputVoxel;
    }
    else {
        // std::cout << "Voxel position out of bounds" << std::endl;
        Voxel outputVoxel;
        outputVoxel.position = query;
        outputVoxel.distanceToObstacle = -1.0;
        return outputVoxel;
    }
}

void CostMap3D::GetVoxelNeighbors3D(float x, float y, float z, std::vector<Voxel> &neighbors, std::vector<float> &distances, std::vector<int> &indices)
{
    // Get the 26 voxel neighbors of queried position
    neighbors.resize(26);
    distances.resize(26);
    indices.resize(26);
    int neighborId = 0;
    float n_x, n_y, n_z; // neighbor position coordinates
    for (int i=-1; i<2; i++) {
        n_x = x + i*voxelSize;
        for (int j=-1; j<2; j++) {
            n_y = y + j*voxelSize;
            for (int k=-1; k<2; k++) {
                n_z = z + k*voxelSize;
                if (!((i==0) && (j==0) && (k==0))) {
                    Voxel neighbor = Query(n_x, n_y, n_z);
                    neighbors[neighborId] = neighbor;
                    distances[neighborId] = (std::sqrt(std::abs(i) + std::abs(j) + std::abs(k)))*voxelSize;
                    indices[neighborId] = ConvertPositionToIndex(neighbor.position);
                    neighborId++;
                }
            }
        }
    }
    return;
}