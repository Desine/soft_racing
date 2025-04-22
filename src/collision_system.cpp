// #include "collision_system.hpp"
// #include <algorithm>

// void DetectPointsLinesCollisions(
//     const PointMasses &points,
//     const std::vector<uint32_t> &pointIndices,
//     const PointMasses &lines,
//     const std::vector<uint32_t> &lineIndices,
//     std::vector<CollisionConstraint> &outConstraints,
//     float compliance,
//     float minDistance)
// {
//     const auto &pointPositions = points.positions;
//     const auto &segmentPositions = lines.positions;

//     for (uint32_t pi : pointIndices)
//     {
//         glm::vec2 p = pointPositions[pi];

//         for (size_t i = 0; i + 1 < lineIndices.size(); i += 2)
//         {
//             uint32_t li1 = lineIndices[i];
//             uint32_t li2 = lineIndices[i + 1];

//             glm::vec2 a = segmentPositions[li1];
//             glm::vec2 b = segmentPositions[li2];
//             glm::vec2 ab = b - a;

//             float len2 = glm::dot(ab, ab);
//             if (len2 == 0.0f)
//                 continue;

//             float t = glm::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f);
//             glm::vec2 proj = a + t * ab;
//             glm::vec2 offset = p - proj;

//             float dist = glm::length(offset);
//             if (dist < minDistance && dist > 1e-6f)
//             {
//                 glm::vec2 normal = glm::normalize(offset);
//                 outConstraints.push_back({pi, li1, li2,
//                                           normal,
//                                           minDistance - dist,
//                                           compliance,
//                                           0.0f});
//             }
//         }
//     }
// }
// void DetectSoftBodyCollisions(SoftBody &A, SoftBody &B, float compliance = 1e-6f, float minDistance = 0.01f)
// {
//     DetectPointsLinesCollisions(
//         A.pointMasses,
//         A.collisionPoints,
//         B.pointMasses,
//         B.collisionLines,
//         A.collisionConstraints,
//         compliance,
//         minDistance);

//     DetectPointsLinesCollisions(
//         B.pointMasses,
//         B.collisionPoints,
//         A.pointMasses,
//         A.collisionLines,
//         B.collisionConstraints,
//         compliance,
//         minDistance);
// }

