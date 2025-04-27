#include <fstream>
#include <iostream>
#include "json.hpp"
#include <glm/glm.hpp>
#include "soft_body.hpp"

using json = nlohmann::json;

SoftBody LoadSoftBodyFromFile(const std::string &filename)
{
    SoftBody softBody;

    std::ifstream file(filename);
    if (!file.is_open())
        throw std::runtime_error("Failed to open file: " + filename);

    json j;
    file >> j;

    // PointMasses
    for (const auto &pos : j["pointMasses"]["positions"])
        softBody.pointMasses.positions.emplace_back(pos[0], pos[1]);

    int pointCount = softBody.pointMasses.positions.size();
    softBody.pointMasses.prevPositions.resize(pointCount, glm::vec2(0.0f, 0.0f));
    softBody.pointMasses.velocities.resize(pointCount, glm::vec2(0.0f, 0.0f));

    for (const auto &mass : j["pointMasses"]["masses"])
    {
        float m = mass.get<float>();
        softBody.pointMasses.inverseMasses.push_back(m > 0.0f ? 1.0f / m : 0.0f);
    }

    // DistanceConstraints
    for (const auto &dc : j["distanceConstraints"])
    {
        DistanceConstraint constraint;
        float compliance = dc.value("compliance", 0.0f);
        if (dc.contains("restDistance"))
            constraint = CreateDistanceConstraint(softBody.pointMasses.positions, dc["i1"], dc["i2"], dc["restDistance"], compliance);
        else
            constraint = CreateDistanceConstraint(softBody.pointMasses.positions, dc["i1"], dc["i2"], compliance);
        softBody.distanceConstraints.push_back(constraint);
    }

    // VolumeConstraints
    for (const auto &vc : j["volumeConstraints"])
    {
        VolumeConstraint constraint;

        constraint.compliance = vc.value("compliance", 0.0f);
        for (const auto &idx : vc["indices"])
            constraint.indices.push_back(idx);

        constraint.restVolume = vc.value("restVolume", ComputePolygonArea(softBody.pointMasses.positions, constraint.indices));
        softBody.volumeConstraints.push_back(constraint);
    }

    // ShapeMatchingConstraints
    for (const auto &smc : j["shapeMatchingConstraints"])
    {
        ShapeMatchingConstraint constraint;
        constraint.compliance = smc.value("compliance", 0.0f);

        std::vector<float> inverseMasses;

        for (const auto &idx : smc["indices"]){
            constraint.indices.push_back(idx);
            inverseMasses.push_back(softBody.pointMasses.inverseMasses[idx]);
        }

        for (const auto &sp : smc["startPositions"])
            constraint.startPositions.emplace_back(sp[0], sp[1]);

        constraint.startCenterMass = ComputeMassCenter(softBody.pointMasses.positions, inverseMasses);

        softBody.shapeMatchingConstraints.push_back(constraint);
    }

    // PinConstraints
    for (const auto &pc : j["pinConstraints"])
    {
        PinConstraint constraint;
        constraint.index = pc["index"];
        constraint.targetPosition = glm::vec2(pc["targetPosition"][0], pc["targetPosition"][1]);
        constraint.compliance = pc.value("compliance", 0.0f);
        softBody.pinConstraints.push_back(constraint);
    }

    // Collision data
    if (j.contains("collisionPoints"))
    {
        for (const auto &idx : j["collisionPoints"])
        {
            softBody.collisionPoints.push_back(idx);
        }
    }
    if (j.contains("collisionShape"))
    {
        for (const auto &idx : j["collisionShape"])
        {
            softBody.collisionShape.push_back(idx);
        }
    }

    // center SoftBody on Vector2(0, 0);
    glm::vec2 geometryCenter = ComputeGeometryCenter(softBody.pointMasses.positions);
    for (auto &p : softBody.pointMasses.positions)
        p -= geometryCenter;

    softBody.pointMasses.prevPositions = softBody.pointMasses.positions;

    return softBody;
}
