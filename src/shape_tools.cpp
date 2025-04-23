#include "shape_tools.hpp"
#include <cmath>
#include <algorithm>

// void CreateWheel(
//     const glm::vec2 &center,
//     float wheelRadius,
//     float diskMass,
//     float tireMass,
//     float tireRatio,
//     float diskHubCompliance,
//     float diskRimCompliance,
//     float tireBodyCompliance,
//     float tireTreadCompliance,
//     float tirePressureCompliance,
//     float tirePressure,
//     int radialSegments)
// {
//     radialSegments = std::clamp(radialSegments, 3, 30);
//     tireRatio = std::clamp(tireRatio, 0.1f, 0.7f);

//     wheel.Clear();

//     float diskPointMass = diskMass / (radialSegments + 1);
//     float tirePointMass = tireMass / radialSegments;

//     float innerRadius = wheelRadius * (1.0f - tireRatio);
//     float outerRadius = wheelRadius;

//     float angleStep = 2.0f * float(M_PI) / radialSegments;
//     float halfAngleStep = 0.5f * angleStep;

//     // центр — одна точка
//     int centerId = wheel.pointMasses.Add(center, diskPointMass);

//     // preallocate ids
//     std::vector<int> innerIds(radialSegments);
//     std::vector<int> outerIds(radialSegments);

//     for (int i = 0; i < radialSegments; ++i) {
//         float angle = i * angleStep;

//         glm::vec2 dirDisk = glm::vec2(cosf(angle), sinf(angle));
//         glm::vec2 diskPos = center + dirDisk * innerRadius;
//         innerIds[i] = wheel.pointMasses.Add(diskPos, diskPointMass);

//         float angleOuter = angle - halfAngleStep;
//         glm::vec2 dirTire = glm::vec2(cosf(angleOuter), sinf(angleOuter));
//         glm::vec2 tirePos = center + dirTire * outerRadius;
//         outerIds[i] = wheel.pointMasses.Add(tirePos, tirePointMass);
//     }

//     for (int i = 0; i < radialSegments; ++i) {
//         int next = (i + 1) % radialSegments;

//         // диск к центру
//         wheel.distanceConstraints.Add(wheel.pointMasses, centerId, innerIds[i], glm::distance(wheel.pointMasses.position[centerId], wheel.pointMasses.position[innerIds[i]]), diskHubCompliance);

//         // диск по кругу
//         wheel.distanceConstraints.Add(wheel.pointMasses, innerIds[i], innerIds[next], glm::distance(wheel.pointMasses.position[innerIds[i]], wheel.pointMasses.position[innerIds[next]]), diskRimCompliance);

//         // диск ↔ покрышка
//         wheel.distanceConstraints.Add(wheel.pointMasses, innerIds[i], outerIds[i], glm::distance(wheel.pointMasses.position[innerIds[i]], wheel.pointMasses.position[outerIds[i]]), tireBodyCompliance);
//         wheel.distanceConstraints.Add(wheel.pointMasses, innerIds[i], outerIds[next], glm::distance(wheel.pointMasses.position[innerIds[i]], wheel.pointMasses.position[outerIds[next]]), tireBodyCompliance);

//         // покрышка по кругу
//         wheel.distanceConstraints.Add(wheel.pointMasses, outerIds[i], outerIds[next], glm::distance(wheel.pointMasses.position[outerIds[i]], wheel.pointMasses.position[outerIds[next]]), tireTreadCompliance);
//     }

//     // Объем диска (чтобы держал форму)
//     wheel.volumeConstraints.Add(wheel.pointMasses, innerIds); // auto-computed restVolume в .Add

//     // Объем покрышки + давление
//     int idx = wheel.volumeConstraints.Add(wheel.pointMasses, outerIds, 0.0f, tirePressureCompliance);
//     wheel.volumeConstraints.restVolume[idx] *= tirePressure;
// }

std::vector<glm::vec2> CreatePoigonPositions(int segments, float radius, glm::vec2 origin)
{
    if (segments < 3)
        segments = 3;

    std::vector<glm::vec2> positions;

    float angleStep = 2.0f * float(M_PI) / segments;
    for (int i = 0; i < segments; ++i)
    {
        float angle = i * angleStep;
        glm::vec2 dir = glm::vec2(cosf(angle), sinf(angle));
        glm::vec2 pos = dir * radius + origin;
        positions.push_back(pos);
    }
    return positions;
}
void AddDistanceConstraintsToLoop(SoftBody &softBody, float compliance)
{
    int pointCount = softBody.pointMasses.positions.size();
    for (int i = 0; i < pointCount; ++i)
        softBody.distanceConstraints.push_back(CreateDistanceConstraint(softBody.pointMasses.positions, i, (i + 1) % pointCount, compliance));
}
void AddVolumeConstraintToLoop(SoftBody &softBody, float compliance)
{
    VolumeConstraint vc;
    for (int i = 0; i < softBody.pointMasses.positions.size(); ++i)
        vc.indices.push_back(i);

    vc.restVolume = ComputePolygonArea(softBody.pointMasses.positions, vc.indices);
    vc.compliance = compliance;
    softBody.volumeConstraints.push_back(vc);
}
void AddCollisionPointsToLoop(SoftBody &softBody)
{
    for (int i = 0; i < softBody.pointMasses.positions.size(); ++i)
        softBody.collisionPoints.push_back(i);
}
void AddCollisionShapeToLoop(SoftBody &softBody)
{
    for (int i = 0; i < softBody.pointMasses.positions.size(); ++i)
        softBody.collisionShape.push_back(i);
}