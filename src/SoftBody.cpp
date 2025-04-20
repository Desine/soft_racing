#include "SoftBody.hpp"
#include "Level.hpp"
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <iostream>
#include <ctime>
#include <random>
#include <algorithm>

// ----------------------------
// DistanceConstraint
// ----------------------------

int DistanceConstraint::Add(int index1, int index2, float restDistance, float compliance)
{
    this->i1.push_back(index1);
    this->i2.push_back(index2);
    this->restDistance.push_back(restDistance);
    this->compliance.push_back(compliance);
    this->lambda.push_back(0.0f);
    return count++;
}

void DistanceConstraint::Project(PointMass &pointMasses, float deltaTime, bool firstIteration)
{
    for (size_t j = 0; j < i1.size(); ++j)
    {
        int p1 = i1[j];
        int p2 = i2[j];

        glm::vec2 &x1 = pointMasses.position[p1];
        glm::vec2 &x2 = pointMasses.position[p2];

        float w1 = pointMasses.inverseMass[p1];
        float w2 = pointMasses.inverseMass[p2];

        glm::vec2 n = x1 - x2;
        float len = glm::length(n);
        if (len < 1e-6f)
            continue;

        n /= len;

        float C = len - restDistance[j];
        float alphaTilde = compliance[j] / (deltaTime * deltaTime);

        float denom = w1 + w2 + alphaTilde;
        if (denom == 0.0f)
            continue;

        float deltaLambda = (-C - (firstIteration ? 0.0f : alphaTilde * lambda[j])) / denom;
        glm::vec2 deltaX = deltaLambda * n;

        if (w1 > 0.0f)
            x1 += w1 * deltaX;
        if (w2 > 0.0f)
            x2 -= w2 * deltaX;

        if (!firstIteration)
            lambda[j] += deltaLambda;
        else
            lambda[j] = 0.0f;
    }
}

// ----------------------------
// VolumeConstraint
// ----------------------------

int VolumeConstraint::Add(std::vector<int> indices, float restVolume, float compliance)
{
    this->indices.push_back(indices);
    this->restVolume.push_back(restVolume);
    this->compliance.push_back(compliance);
    this->lambda.push_back(0.0f);
    return count++;
}

void VolumeConstraint::Project(PointMass &pointMasses, float deltaTime, bool firstIteration) 
{
    for (size_t j = 0; j < count; ++j)
    {
        const std::vector<int> &vs = indices[j];
        if (vs.size() < 3)
            continue;

        float V = 0.0f;
        for (size_t i = 0; i < vs.size(); ++i)
        {
            glm::vec2 p0 = pointMasses.position[vs[i]];
            glm::vec2 p1 = pointMasses.position[vs[(i + 1) % vs.size()]];
            V += (p0.x * p1.y - p1.x * p0.y);
        }
        V = 0.5f * V;
        float C = V - restVolume[j];

        std::vector<glm::vec2> grads(vs.size());
        float denom = 0.0f;
        for (size_t i = 0; i < vs.size(); ++i)
        {
            glm::vec2 p0 = pointMasses.position[vs[(i + vs.size() - 1) % vs.size()]];
            glm::vec2 p1 = pointMasses.position[vs[(i + 1) % vs.size()]];
            grads[i] = glm::vec2(p1.y - p0.y, p0.x - p1.x) * 0.5f;

            denom += pointMasses.inverseMass[vs[i]] * glm::dot(grads[i], grads[i]);
        }

        float alphaTilde = compliance[j] / (deltaTime * deltaTime);
        float deltaLambda = (-C - (firstIteration ? 0.0f : alphaTilde * lambda[j])) / (denom + alphaTilde);

        for (size_t i = 0; i < vs.size(); ++i)
        {
            int id = vs[i];
            if (pointMasses.inverseMass[id] > 0.0f)
                pointMasses.position[id] += pointMasses.inverseMass[id] * deltaLambda * grads[i];
        }

        if (!firstIteration)
            lambda[j] += deltaLambda;
        else
            lambda[j] = 0.0f;
    }
}

// ----------------------------
// BendingConstraint
// ----------------------------

int BendingConstraint::Add(int index1, int index2, int index3, float restAngle, float compliance)
{
    this->i1.push_back(index1);
    this->i2.push_back(index2);
    this->i3.push_back(index3);
    this->restAngle.push_back(restAngle);
    this->compliance.push_back(compliance);
    this->lambda.push_back(0.0f);
    return count++;
}

void BendingConstraint::Project(PointMass &pointMasses, float deltaTime, bool firstIteration)
{
    for (size_t j = 0; j < i1.size(); ++j)
    {
        int a = i1[j], b = i2[j], c = i3[j];

        glm::vec2 &pa = pointMasses.position[a];
        glm::vec2 &pb = pointMasses.position[b];
        glm::vec2 &pc = pointMasses.position[c];

        glm::vec2 ab = glm::normalize(pb - pa);
        glm::vec2 cb = glm::normalize(pb - pc);

        float angle = acos(glm::clamp(glm::dot(ab, cb), -1.0f, 1.0f));
        float C = angle - restAngle[j];

        glm::vec2 grad_a = glm::vec2(-ab.y, ab.x);
        glm::vec2 grad_c = glm::vec2(cb.y, -cb.x);
        glm::vec2 grad_b = -(grad_a + grad_c);

        float wa = pointMasses.inverseMass[a];
        float wb = pointMasses.inverseMass[b];
        float wc = pointMasses.inverseMass[c];

        float denom = wa * glm::dot(grad_a, grad_a) +
                      wb * glm::dot(grad_b, grad_b) +
                      wc * glm::dot(grad_c, grad_c);

        float alphaTilde = compliance[j] / (deltaTime * deltaTime);
        float deltaLambda = (-C - (firstIteration ? 0.0f : alphaTilde * lambda[j])) / (denom + alphaTilde);

        if (wa > 0.0f)
            pa += wa * deltaLambda * grad_a;
        if (wb > 0.0f)
            pb += wb * deltaLambda * grad_b;
        if (wc > 0.0f)
            pc += wc * deltaLambda * grad_c;

        if (!firstIteration)
            lambda[j] += deltaLambda;
        else
            lambda[j] = 0.0f;
    }
}
/*
void SoftBody::Simulate(float deltaTime, glm::vec2 gravity, const Level &level)
{
    float deltaStep = deltaTime / simulationSubsteps;
    for (int i = 0; i < simulationSubsteps; i++)
    {
        // integration
        for (auto &p : pointMasses)
        {
            p.inverseMass = p.isFixed ? 0.0f : 1.0f / p.mass; // reduce calculation

            p.velocity += deltaStep * gravity;
            p.previousPosition = p.position;
            p.position += deltaStep * p.velocity;
        }

        // detect static collisions
        DetectStaticCollisions(level);

        // solve positions
        SolveDistanceConstraints(deltaStep);
        SolveVolumeConstraints(deltaStep);
        SolveStaticCollisionConstraints();

        // velocity update
        for (auto &p : pointMasses)
        {
            if (p.isFixed)
            continue;
            p.velocity = (p.position - p.previousPosition) / deltaStep;
        }

        // solve velocities
        for (auto &c : staticCollisions)
        {
            PointMass &p = pointMasses[c.pointMassIndex];
            if (p.inverseMass == 0.0f)
            continue;

            float dynamicFriction = 0.3f;

            glm::vec2 normalComponent = glm::dot(p.velocity, c.surfaceNormal) * c.surfaceNormal;
            glm::vec2 tangential = p.velocity - normalComponent;

            p.velocity -= dynamicFriction * tangential;
        }
    }
}


void SoftBody::SolveGroundCollision(const Level &level)
{
    for (auto &p : pointMasses)
    {
        float heightAtPoint = level.GetHeight(p.position.x);
        if (p.position.y < heightAtPoint)
        {
            // p.position.y = heightAtPoint;
            // p.velocity.y = -p.velocity.y * .001f;
            // p.velocity.x = 0;
        }
        if (p.position.y > 2000)
        {
            p.position.y = 2000;
            p.velocity.y = -p.velocity.y;
        }
        if (p.position.x < -100)
        {
            p.position.x = -100;
            p.velocity.x = -p.velocity.x;
        }
    }
}

void SoftBody::Clear()
{
    // pointMasses.clear();
    // distanceConstraints.clear();
    // collisionPointMasses.clear();
    // volumeConstraints.clear();
    // staticCollisions.clear();
}
*/