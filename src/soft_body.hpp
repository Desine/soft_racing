#pragma once
#include <vector>
#include "glm/glm.hpp"
#include <optional>
#include <memory>

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
    float compliance = 0.0f;
    float lambda = 0.0f;
};
struct VolumeConstraint
{
    std::vector<uint32_t> indices;
    float restVolume;
    float compliance = 0.0f;
    float lambda = 0.0f;
};
struct AngleConstraint
{
    uint32_t i1, i2, i3;
    float restAngle;
    float compliance = 0.0f;
    float lambda = 0.0f;
};
struct ShapeMatchingConstraint
{
    std::vector<uint32_t> indices;
    std::vector<glm::vec2> startPositions;
    float compliance = 0.0f;
    float lambda = 0.0f;
};
struct PinConstraint
{
    uint32_t index;
    glm::vec2 targetPosition;
    float compliance = 0.0f;
    float lambda = 0.0f;
};
struct AccelerationConstraint
{
    std::vector<uint32_t> indices;
    glm::vec2 acceleration;
};
struct ForceConstraint
{
    std::vector<uint32_t> indices;
    glm::vec2 force;
};
struct VelocityConstraint
{
    std::vector<uint32_t> indices;
    glm::vec2 velocity;
};
struct AngularAccelerationConstraint
{
    std::vector<uint32_t> indices;
    float acceleration;
    glm::vec2 position;
};
struct AngularForceConstraint
{
    std::vector<uint32_t> indices;
    float force;
    glm::vec2 position;
};
struct AngularVelocityConstraint
{
    std::vector<uint32_t> indices;
    float velocity;
    glm::vec2 position;
};


struct SoftBody
{
    PointMasses pointMasses;
    std::vector<DistanceConstraint> distanceConstraints;
    std::vector<VolumeConstraint> volumeConstraints;
    std::vector<AngleConstraint> angleConstraints;
    std::vector<ShapeMatchingConstraint> shapeMatchingConstraints;
    std::vector<PinConstraint> pinConstraints;
    
    std::vector<AccelerationConstraint> accelerationConstraints;
    std::vector<ForceConstraint> forceConstraints;
    std::vector<VelocityConstraint> VelocityConstraints;
    std::vector<AngularAccelerationConstraint> angularAccelerationConstraints;
    std::vector<AngularForceConstraint> angularForceConstraints;
    std::vector<AngularVelocityConstraint> angularVelocityConstraints;
    std::vector<uint32_t> collisionPoints;
    std::vector<uint32_t> collisionShape;
};

struct RayHit
{
    glm::vec2 point;
    float distance;
    size_t edgeIndex;
};

DistanceConstraint CreateDistanceConstraint(const std::vector<glm::vec2> &positions, uint32_t i1, uint32_t i2, float compliance = 0.0f);
DistanceConstraint CreateDistanceConstraint(const std::vector<glm::vec2> &positions, uint32_t i1, uint32_t i2, float compliance, float restDistance);

AngleConstraint CreateAngleConstraint(const std::vector<glm::vec2> &positions, uint32_t i1, uint32_t i2, uint32_t i3, float compliance = 0.0f);
AngleConstraint CreateAngleConstraint(const std::vector<glm::vec2> &positions, uint32_t i1, uint32_t i2, uint32_t i3, float compliance, float restAngle);

void ResetConstrainsLambdas(SoftBody &softBody);

float ComputePolygonArea(const std::vector<glm::vec2> &positions, const std::vector<uint32_t> &indices);
glm::vec2 ComputeGeometryCenter(const std::vector<glm::vec2> &positions);
glm::vec2 ComputeMassCenter(const std::vector<glm::vec2> &positions, const std::vector<float> &inverseMasses);

std::vector<RayHit> RaycastAllIntersections(const glm::vec2 &origin, const glm::vec2 &direction, SoftBody &body);
std::optional<RayHit> RaycastFirstIntersection(const glm::vec2 &origin, const glm::vec2 &direction, SoftBody &body);

bool PointInPolygon(const glm::vec2 point, const std::vector<glm::vec2> positions);