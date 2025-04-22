// file SoftBody.hpp
#pragma once
#include <vector>
#include "glm/glm.hpp"

struct PointMasses
{
    std::vector<glm::vec2> positions;
    std::vector<glm::vec2> prevPositions;
    std::vector<glm::vec2> velocities;
    std::vector<float> inverseMasses;
};
struct DistanceConstraint
{
    uint32_t i1, i2;
    float restDistance;
    float compliance;
    float lambda;
};
struct VolumeConstraint
{
    std::vector<uint32_t> indices;
    float restVolume;
    float compliance;
    float lambda;
};
struct PinConstraint
{
    uint32_t index;
    glm::vec2 targetPosition;
    float compliance;
    float lambda = 0.0f;
};
struct CollisionConstraint
{
    uint32_t pointIndex;
    glm::vec2 normal;
    glm::vec2 contactPoint;
    float compliance;
    float lambda;
};

struct SoftBody
{
    PointMasses pointMasses;
    std::vector<DistanceConstraint> distanceConstraints;
    std::vector<VolumeConstraint> volumeConstraints;
    std::vector<PinConstraint> pinConstraints;
    std::vector<CollisionConstraint> collisionConstraints;
};

float ComputePolygonArea(const std::vector<glm::vec2> &positions, const std::vector<uint32_t> &indices);
glm::vec2 GetGeometryCenter(PointMasses &pointMasses);