
#include "CarTools.hpp"
#include <cmath>

void CreateWheel(
    SoftBody &wheel,
    const glm::vec2 &center,
    float wheelRadius,
    float diskMass,
    float tireMass,
    float tireRatio,
    float tireBodyCompliance,
    float tireTreadCompliance,
    float tirePressure,
    int radialSegments)
{
    /*
    параметры:
    wheelRadius - радиус колеса вместе с диском и покрышкой
    tireRatio - толщина покрышки в процентах от радиуса колеса [0.1; 0.7]
    tireBodyCompliance
    tireTreadCompliance
    tirePressure
    radialSegments - [3; 30]


    расставить точки:
    диск - 1 точка в центре
    диск - radialSegments точки соеденены в кольцо и с центральной точкой
    покрышка - radialSegments точки соеденены в кольцо, compliance = tireTreadCompliance
    покрышка - radialSegments точки соеденены с с radialSegments диска, compliance = tireBodyCompliance
    покрышка - кольцо соединено VolumeConstraint с restVolume = currentVolume * tirePressure
    */

    // safety clamp
    radialSegments = std::clamp(radialSegments, 3, 30);
    tireRatio = std::clamp(tireRatio, 0.1f, 0.7f);

    // SoftBody clear
    wheel.Clear();

    // prepare values
    float diskPointMass = diskMass / (radialSegments + 1);
    float tirePointMass = tireMass / radialSegments;

    float innerRadius = wheelRadius * (1.0f - tireRatio);
    float outerRadius = wheelRadius;
    float angleStep = 2 * M_PI / radialSegments;

    wheel.AddPoint(center, diskPointMass);

    std::vector<int> innerIds;
    std::vector<int> outerIds;
    for (int i = 0; i < radialSegments; i++)
    {
        float angle = i * angleStep;
        glm::vec2 dir = glm::vec2(std::cos(angle), std::sin(angle));
        glm::vec2 diskPos = center + dir * innerRadius;
        glm::vec2 tirePos = center + dir * outerRadius;

        innerIds.push_back(wheel.AddPoint(diskPos, diskPointMass));
        outerIds.push_back(wheel.AddPoint(tirePos, tirePointMass));
    }

    std::reverse(innerIds.begin(),innerIds.end());
    for (int i = 0; i < radialSegments; i++)
    {
        wheel.AddDistanceConstraint(0, innerIds[i], .01f);
        wheel.AddDistanceConstraint(innerIds[i], outerIds[radialSegments - 1 - i]);
        wheel.AddDistanceConstraint(innerIds[i], innerIds[(i + 1) % radialSegments], tireBodyCompliance);
        wheel.AddDistanceConstraint(outerIds[i], outerIds[(i + 1) % radialSegments], tireTreadCompliance);
    }

    wheel.AddVolumeConstraint(outerIds, tireTreadCompliance);
    wheel.volumeConstraints.back().restVolume *= tirePressure;
}
