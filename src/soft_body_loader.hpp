#pragma once
#include <fstream>
#include <iostream>
#include "json.hpp"
#include <glm/glm.hpp>
#include "soft_body.hpp"
#include "shape_tools.hpp"
#include "car.hpp"

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
            constraint = CreateDistanceConstraint(softBody.pointMasses.positions, dc["i1"], dc["i2"], compliance, dc["restDistance"]);
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

    // AngleConstraints
    for (const auto &dc : j["angleConstraints"])
    {
        AngleConstraint constraint;
        float compliance = dc.value("compliance", 0.0f);
        if (dc.contains("restAngle"))
            constraint = CreateAngleConstraint(softBody.pointMasses.positions, dc["i1"], dc["i2"], dc["i3"], compliance, dc["restAngle"]);
        else
            constraint = CreateAngleConstraint(softBody.pointMasses.positions, dc["i1"], dc["i2"], dc["i3"], compliance);
        softBody.angleConstraints.push_back(constraint);
    }

    // ShapeMatchingConstraints
    for (const auto &smc : j["shapeMatchingConstraints"])
    {
        ShapeMatchingConstraint constraint;
        constraint.compliance = smc.value("compliance", 0.0f);

        std::vector<float> inverseMasses;

        for (const auto &idx : smc["indices"])
        {
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

    softBody.pointMasses.prevPositions = softBody.pointMasses.positions;

    return softBody;
}

Car LoadCarFromFile(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
        throw std::runtime_error("Failed to open car file: " + filename);

    json j;
    file >> j;

    Car car;

    // Load body
    std::shared_ptr<SoftBody> bodyPtr = std::make_shared<SoftBody>(LoadSoftBodyFromFile(j["bodyFile"]));
    car.body = bodyPtr;

    const auto &wheelDefs = j["wheels"];
    for (const auto &w : wheelDefs)
    {
        glm::vec2 position(w["position"][0], w["position"][1]);
        float radius = w.value("radius", 50);
        float diskMass = w.value("diskMass", 10.0f);
        float tireMass = w.value("tireMass", 5.0f);
        float tireRatio = w.value("tireRatio", 0.5f);
        float diskHubCompliance = w.value("diskHubCompliance", 0.0f);
        float diskRimCompliance = w.value("diskRimCompliance", 0.0f);
        float tireBodyCompliance = w.value("tireBodyCompliance", 0.001f);
        float tireTreadCompliance = w.value("tireTreadCompliance", 0.001f);
        float tirePressureCompliance = w.value("tirePressureCompliance", 0.0f);
        float tirePressure = w.value("tirePressure", 1.0f);
        int segments = w.value("segments", 3);

        SoftBody wheel = CreateWheel(
            position,
            radius,
            diskMass,
            tireMass,
            tireRatio,
            diskHubCompliance,
            diskRimCompliance,
            tireBodyCompliance,
            tireTreadCompliance,
            tirePressureCompliance,
            tirePressure,
            segments);

        std::shared_ptr<SoftBody> wheelPtr = std::make_shared<SoftBody>(std::move(wheel));
        car.wheels.push_back(wheelPtr);

        const auto &bodyIndices = w["bodyIndices"];
        const auto &jointCompliance = w.value("jointCompliance", 0.0f);
        for (const auto &i : bodyIndices)
        {
            auto djPtr = std::make_shared<DistanceJoint>();
            djPtr->softBody1 = wheelPtr;
            djPtr->softBody2 = bodyPtr;
            uint32_t bodyIndex = i.get<uint32_t>();
            djPtr->index1 = 0;
            djPtr->index2 = bodyIndex;

            const glm::vec2 &p1 = wheelPtr->pointMasses.positions[0];
            const glm::vec2 &p2 = bodyPtr->pointMasses.positions[bodyIndex];
            float restDistance = glm::length(p1 - p2);
            djPtr->restDistance = restDistance;
            djPtr->compliance = jointCompliance;
            djPtr->lambda = 0.0f;
            car.distanceJoints.push_back(djPtr);
        }

        // if (w.contains("motor"))
        // {
        //     const auto &m = w["motor"];
        //     auto mj = std::make_shared<MotorJoint>(wheelPtr, bodyPtr);

        //     mj->targetAngularVelocity = m.value("targetAngularVelocity", 0.0f);
        //     mj->torque = m.value("torque", 0.0f);
        //     mj->compliance = m.value("compliance", 0.0f);
        //     mj->lambda = 0.0f;

        //     // Индексы точек на колесе — все или только некоторые
        //     for (uint32_t i = 0; i < wheelPtr->pointMasses.positions.size(); ++i)
        //         mj->indices1.push_back(i);

        //     // Индексы точек тела (заданы в JSON как motor.bodyIndices)
        //     if (m.contains("bodyIndices"))
        //     {
        //         for (const auto &i : m["bodyIndices"])
        //             mj->indices2.push_back(i.get<uint32_t>());
        //     }
        //     else
        //     {
        //         for (uint32_t i = 0; i < bodyPtr->pointMasses.positions.size(); ++i)
        //             mj->indices2.push_back(i);
        //     }

        //     car.motorJoints.push_back(mj);
        // }
    }

    return car;
}
