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