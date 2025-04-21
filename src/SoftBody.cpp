//file SoftBody.cpp
#include "SoftBody.hpp"

float ComputePolygonArea(const std::vector<glm::vec2>& positions, const std::vector<uint32_t>& indices)
{
    float area = 0.0f;
    size_t N = indices.size();
    for (size_t i = 0; i < N; ++i)
    {
        const glm::vec2& p0 = positions[indices[i]];
        const glm::vec2& p1 = positions[indices[(i + 1) % N]];
        area += p0.x * p1.y - p1.x * p0.y;
    }
    return 0.5f * area;
}


