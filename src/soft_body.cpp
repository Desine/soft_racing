#include "soft_body.hpp"
#include "utils.hpp"

#include <algorithm>
#include <limits>
#include <iostream>

DistanceConstraint CreateDistanceConstraint(const std::vector<glm::vec2> &positions, uint32_t i1, uint32_t i2, float compliance)
{
    DistanceConstraint constraint;
    constraint.i1 = i1;
    constraint.i2 = i2;
    constraint.restDistance = glm::distance(positions[i1], positions[i2]);
    constraint.compliance = compliance;
    return constraint;
}
DistanceConstraint CreateDistanceConstraint(const std::vector<glm::vec2> &positions, uint32_t i1, uint32_t i2, float compliance, float restDistance)
{
    DistanceConstraint constraint = CreateDistanceConstraint(positions, i1, i2, compliance);
    constraint.restDistance = restDistance;
    return constraint;
}
AngleConstraint CreateAngleConstraint(const std::vector<glm::vec2> &positions, uint32_t i1, uint32_t i2, uint32_t i3, float compliance)
{
    AngleConstraint constraint;
    constraint.i1 = i1;
    constraint.i2 = i2;
    constraint.i3 = i3;
    constraint.compliance = compliance;
    constraint.restAngle = CalculateAngle(positions[i1], positions[i2], positions[i3]);
    return constraint;
}

AngleConstraint CreateAngleConstraint(const std::vector<glm::vec2> &positions, uint32_t i1, uint32_t i2, uint32_t i3, float compliance, float restAngle)
{
    AngleConstraint constraint = CreateAngleConstraint(positions, i1, i2, i3, compliance);
    constraint.restAngle = restAngle;
    return constraint;
}

void ResetConstrainsLambdas(SoftBody &softBody)
{
    for (auto &c : softBody.distanceConstraints)
        c.lambda = 0.0f;

    for (auto &c : softBody.volumeConstraints)
        c.lambda = 0.0f;

    for (auto &c : softBody.angleConstraints)
        c.lambda = 0.0f;

    for (auto &c : softBody.shapeMatchingConstraints)
        c.lambda = 0.0f;

    for (auto &c : softBody.pinConstraints)
        c.lambda = 0.0f;
}

float ComputePolygonArea(const std::vector<glm::vec2> &positions, const std::vector<uint32_t> &indices)
{
    float area = 0.0f;
    size_t N = indices.size();
    for (size_t i = 0; i < N; ++i)
    {
        const glm::vec2 &p0 = positions[indices[i]];
        const glm::vec2 &p1 = positions[indices[(i + 1) % N]];
        area += Cross2D(p0, p1);
    }
    return 0.5f * area;
}

glm::vec2 ComputeGeometryCenter(const std::vector<glm::vec2> &positions)
{
    glm::vec2 center = glm::vec2(0, 0);
    for (auto &p : positions)
        center += p;

    return center /= positions.size();
}
glm::vec2 ComputeMassCenter(const std::vector<glm::vec2> &positions, const std::vector<float> &inverseMasses)
{
    glm::vec2 massCenter = glm::vec2(0.0f);
    float totalMass = 0.0f;

    for (size_t i = 0; i < positions.size(); ++i)
    {
        float w = inverseMasses[i];
        float m = (w > 0.0f) ? 1.0f / w : 0.0f;
        massCenter += positions[i] * m;
        totalMass += m;
    }

    if (totalMass > 0.0f)
        massCenter /= totalMass;

    return massCenter;
}

std::vector<RayHit> RaycastAllIntersections(const glm::vec2 &origin, const glm::vec2 &direction, SoftBody &body)
{
    std::vector<RayHit> hits;
    const auto &shape = body.collisionShape;
    const auto &positions = body.pointMasses.positions;

    for (size_t i = 0; i < shape.size(); ++i)
    {
        glm::vec2 p1 = positions[shape[i]];
        glm::vec2 p2 = positions[shape[(i + 1) % shape.size()]];
        glm::vec2 edge = p2 - p1;
        glm::vec2 normal(-direction.y, direction.x);

        float denom = glm::dot(edge, normal);
        if (std::abs(denom) < 1e-6f)
            continue;

        float t = glm::dot(origin - p1, normal) / denom;
        float u = Cross2D(origin - p1, edge) / Cross2D(direction, edge);

        if (u >= 0 && t >= 0 && t <= 1)
        {
            RayHit hit;
            hit.point = origin + direction * u;
            hit.distance = u;
            hit.edgeIndex = i;
            hits.push_back(hit);
        }
    }

    return hits;
}

std::optional<RayHit> RaycastFirstIntersection(const glm::vec2 &origin, const glm::vec2 &direction, SoftBody &body)
{
    auto hits = RaycastAllIntersections(origin, direction, body);
    if (hits.empty())
        return std::nullopt;

    return *std::min_element(hits.begin(), hits.end(),
                             [](const RayHit &a, const RayHit &b)
                             {
                                 return a.distance < b.distance;
                             });
}
