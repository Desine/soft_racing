#pragma once
#include "glm/glm.hpp"

inline float Cross2D(const glm::vec2 &a, const glm::vec2 &b)
{
    return a.x * b.y - a.y * b.x;
}
inline glm::vec2 Perp2D(const glm::vec2 &v)
{
    return glm::vec2(v.y, -v.x);
}
inline glm::vec2 Perp2D(const glm::vec2 &a, const glm::vec2 &b)
{
    return glm::vec2(a.y, -a.x) - glm::vec2(b.y, -b.x);
}
inline float CalculateAngle(glm::vec2 p1, glm::vec2 p2, glm::vec2 p3)
{
    glm::vec2 v1 = p1 - p2;
    glm::vec2 v2 = p3 - p2;

    float dotProduct = glm::dot(v1, v2);
    float magnitudeV1 = glm::length(v1);
    float magnitudeV2 = glm::length(v2);
    if (magnitudeV1 == 0.0f || magnitudeV2 == 0.0f)
        return 0.0f;

    float cosTheta = glm::clamp(dotProduct / (magnitudeV1 * magnitudeV2), -1.0f, 1.0f);
    float angle = std::acos(cosTheta);

    if (Cross2D(v1, v2) < 0)
        angle = 2.0f * float(M_PI) - angle;

    return angle;
}