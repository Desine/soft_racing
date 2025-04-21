#include "CarTools.hpp"
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





// void PhysicsScene::AddCloth(glm::vec2 origin, int w, int h, float spacing, float mass, float compliance) {
//     int* indices = new int[w * h];

//     for (int y = 0; y < h; ++y)
//         for (int x = 0; x < w; ++x) {
//             glm::vec2 pos = origin + glm::vec2(x * spacing, y * spacing);
//             indices[y * w + x] = pointMasses.Add(pos, mass);
//         }

//     for (int y = 0; y < h; ++y)
//         for (int x = 0; x < w; ++x) {
//             int id = indices[y * w + x];
//             if (x + 1 < w) distanceConstraints.Add(id, indices[y * w + (x + 1)], spacing, compliance);
//             if (y + 1 < h) distanceConstraints.Add(id, indices[(y + 1) * w + x], spacing, compliance);
//         }

//     delete[] indices;
// }