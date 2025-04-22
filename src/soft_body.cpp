// file soft_body.cpp
#include "soft_body.hpp"

float ComputePolygonArea(const std::vector<glm::vec2> &positions, const std::vector<uint32_t> &indices)
{
    float area = 0.0f;
    size_t N = indices.size();
    for (size_t i = 0; i < N; ++i)
    {
        const glm::vec2 &p0 = positions[indices[i]];
        const glm::vec2 &p1 = positions[indices[(i + 1) % N]];
        area += p0.x * p1.y - p1.x * p0.y;
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
