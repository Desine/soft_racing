#pragma once

#include <vector>
#include "glm/glm.hpp"



class Level
{
public:
    Level(unsigned int seed);

    std::vector<glm::vec2> GetPoints(float carPositionX, float width, float precision) const;
    float GetHeight(float positionX) const;
    glm::vec2 GetNormal(float positionX) const;
    glm::vec2 ProjectPointToSurface(const glm::vec2& point) const;
    
    private:
        unsigned int sid;
        std::vector<std::pair<float, float>> noiseComponents; // (frequency, amplitude)
};