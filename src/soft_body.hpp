// file soft_body.hpp
#pragma once
#include <vector>
#include "glm/glm.hpp"
#include <optional>

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
    float lambda;
};

struct SoftBody
{
    PointMasses pointMasses;
    std::vector<DistanceConstraint> distanceConstraints;
    std::vector<VolumeConstraint> volumeConstraints;
    std::vector<PinConstraint> pinConstraints;
    std::vector<uint32_t> collisionPoints;
    std::vector<uint32_t> collisionShape;
};

struct RayHit
{
    glm::vec2 point;
    float distance;
    size_t edgeIndex;
};

float ComputePolygonArea(const std::vector<glm::vec2> &positions, const std::vector<uint32_t> &indices);
glm::vec2 GetGeometryCenter(PointMasses &pointMasses);

std::vector<RayHit> RaycastAllIntersections(const glm::vec2 &origin, const glm::vec2 &direction, const SoftBody &body);
std::optional<RayHit> RaycastFirstIntersection(const glm::vec2 &origin, const glm::vec2 &direction, const SoftBody &body);