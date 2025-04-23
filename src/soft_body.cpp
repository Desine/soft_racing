// file soft_body.cpp
#include "soft_body.hpp"
#include "utils.hpp"
#include <algorithm>
#include <limits>

#include <iostream>

float ComputePolygonArea(const std::vector<glm::vec2> &positions, const std::vector<uint32_t> &indices)
{
    float area = 0.0f;
    size_t N = indices.size();
    for (size_t i = 0; i < N; ++i)
    {
        const glm::vec2 &p0 = positions[indices[i]];
        const glm::vec2 &p1 = positions[indices[(i + 1) % N]];
        area += Cross2D(p1, p0);
    }
    return 0.5f * area;
}

glm::vec2 GetGeometryCenter(PointMasses &pointMasses)
{
    glm::vec2 center = glm::vec2(0, 0);
    for (auto &p : pointMasses.positions)
        center += p;

    return center /= pointMasses.positions.size();
}

std::vector<RayHit> RaycastAllIntersections(const glm::vec2 &origin, const glm::vec2 &direction, const SoftBody &body)
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

std::optional<RayHit> RaycastFirstIntersection(const glm::vec2 &origin, const glm::vec2 &direction, const SoftBody &body)
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
