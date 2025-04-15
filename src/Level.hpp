#pragma once

#include <vector>

struct Vec2
{
    float x, y;
};

class Level
{
public:
    Level(unsigned int seed);

    std::vector<Vec2> GetPoints(float carPositionX, float width, float precision) const;

private:
    unsigned int sid;
    std::vector<std::pair<float, float>> noiseComponents; // (frequency, amplitude)
};