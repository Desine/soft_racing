#include "glm/glm.hpp"
#include "soft_body.hpp"
#include "joint_system.hpp"
#include <memory>

class Car
{
public:
    std::shared_ptr<SoftBody> body;
    std::vector<std::shared_ptr<SoftBody>> wheels;
    std::vector<std::shared_ptr<DistanceJoint>> distanceJoints;
};