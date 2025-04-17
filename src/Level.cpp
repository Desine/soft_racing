#include "Level.hpp"
#include <cmath>
#include <random>

Level::Level(unsigned int seed)
    : sid(seed)
{
    noiseComponents = {
        {0.005f, 40.0f},
        {0.02f, 10.0f},
        {0.1f, 3.0f}
    };
}

std::vector<glm::vec2> Level::GetPoints(float carPositionX, float width, float precision) const
{
    std::vector<glm::vec2> result;

    float startX = carPositionX - width / 2.0f;
    float endX = carPositionX + width / 2.0f;

    for (float x = startX; x <= endX; x += precision)
    {
        float y = GetHeight(x);
        result.push_back(glm::vec2{x, y});
    }

    return result;
}

float Level::GetHeight(float positionX) const
{
    std::mt19937 rng(sid); // генератор случайных чисел по сиду
    std::uniform_real_distribution<float> noiseOffsetDist(0.0f, 1000.0f);

    std::vector<float> offsets;
    for (size_t i = 0; i < noiseComponents.size(); ++i)
        offsets.push_back(noiseOffsetDist(rng));

    float y = 0.0f;
    for (size_t i = 0; i < noiseComponents.size(); ++i)
    {
        float freq = noiseComponents[i].first;
        float amp = noiseComponents[i].second;
        float offset = offsets[i];

        y += std::sin((positionX + offset) * freq) * amp;
    }

    return y;
}