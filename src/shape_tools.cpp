#include "shape_tools.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

SoftBody CreateWheel(
    const glm::vec2 &center,
    float wheelRadius,
    float diskMass,
    float tireMass,
    float tireRatio,
    float diskHubCompliance,
    float diskRimCompliance,
    float tireBodyCompliance,
    float tireTreadCompliance,
    float tirePressureCompliance,
    float tirePressure,
    int radialSegments)
{
    if (radialSegments < 3)
        radialSegments = 3;

    radialSegments = std::clamp(radialSegments, 3, 30);
    tireRatio = std::clamp(tireRatio, 0.1f, 0.7f);

    SoftBody wheel;
    PointMasses &pm = wheel.pointMasses;

    float diskPointMass = diskMass / (radialSegments + 1);
    float tirePointMass = tireMass / radialSegments;

    float diskRadius = wheelRadius * (1.0f - tireRatio);

    // CreatePoigonPositions for tire should be offset
    float rotate_angle = float(M_PI) / radialSegments;

    // центральная точка
    pm.positions.push_back(center);
    pm.prevPositions.push_back(center);
    pm.velocities.push_back({0.0f, 0.0f});
    pm.inverseMasses.push_back(1.0f / diskPointMass);

    // // disk
    // for (auto p : CreatePoigonPositions(radialSegments, diskRadius, center))
    // {
    //     pm.positions.push_back(p);
    //     pm.prevPositions.push_back(p);
    //     pm.velocities.push_back({0.0f, 0.0f});
    //     pm.inverseMasses.push_back(1.0f / diskPointMass);
    // }

    // tire
    for (auto p : CreatePoigonPositions(radialSegments, wheelRadius, center)) // , -rotate_angle
    {
        pm.positions.push_back(p);
        pm.prevPositions.push_back(p);
        pm.velocities.push_back({0.0f, 0.0f});
        pm.inverseMasses.push_back(1.0f / tirePointMass);
    }

    VolumeConstraint vc_tire;

    std::vector<DistanceConstraint> &dc = wheel.distanceConstraints;

    int diskStart = 1;
    int tireStart = 1 + radialSegments;

    for (int i = 0; i < radialSegments; i++)
    {
        int next = (i + 1) % radialSegments;

        // dc.push_back(CreateDistanceConstraint(pm.positions, 0, diskStart + i, diskHubCompliance));
        // dc.push_back(CreateDistanceConstraint(pm.positions, diskStart + i, diskStart + next, diskRimCompliance));
        // dc.push_back(CreateDistanceConstraint(pm.positions, tireStart + i, tireStart + next, tireTreadCompliance));
        // dc.push_back(CreateDistanceConstraint(pm.positions, diskStart + i, tireStart + ((i + radialSegments / 4) % radialSegments), tireBodyCompliance));
        // dc.push_back(CreateDistanceConstraint(pm.positions, tireStart + ((i + radialSegments * 3 / 4) % radialSegments), diskStart + i, tireBodyCompliance));

        // dc.push_back(CreateDistanceConstraint(pm.positions, diskStart + i, diskStart + ((i + radialSegments / 3) % radialSegments), diskRimCompliance));
        // // dc.push_back(CreateDistanceConstraint(pm.positions, tireStart + i, tireStart + ((i + radialSegments / 3) % radialSegments), tireBodyCompliance));

        dc.push_back(CreateDistanceConstraint(pm.positions, 0, diskStart + i, tireBodyCompliance));
        dc.push_back(CreateDistanceConstraint(pm.positions, diskStart + i, diskStart + ((i + radialSegments / 3) % radialSegments), tireBodyCompliance));
        dc.push_back(CreateDistanceConstraint(pm.positions, diskStart + i, diskStart + next, tireTreadCompliance));


        // vc_tire.indices.push_back(tireStart + i);
        // wheel.collisionPoints.push_back(tireStart + i);
        // wheel.collisionShape.push_back(tireStart + i);

        vc_tire.indices.push_back(diskStart + i);
        wheel.collisionPoints.push_back(diskStart + i);
        wheel.collisionShape.push_back(diskStart + i);
    }

    vc_tire.compliance = tirePressureCompliance;
    vc_tire.restVolume = tirePressure * ComputePolygonArea(pm.positions, vc_tire.indices);

    // wheel.volumeConstraints.push_back(vc_tire);

    return wheel;
}

std::vector<glm::vec2> CreatePoigonPositions(int segments, float radius, glm::vec2 origin, float rotate_angle)
{
    if (segments < 3)
        segments = 3;

    std::vector<glm::vec2> positions;

    float angleStep = 2.0f * float(M_PI) / segments;
    for (int i = 0; i < segments; ++i)
    {
        float angle = i * angleStep + rotate_angle;
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