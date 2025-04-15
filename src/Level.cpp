#include "Level.hpp"
#include <cmath>
#include <random>

Level::Level(unsigned int seed)
    : sid(seed)
{
    // можно позже загрузить шумовые параметры из файла
    // пока жёстко зададим несколько частот и амплитуд
    noiseComponents = {
        {0.005f, 40.0f},   // длинные холмы
        {0.02f, 10.0f},    // волны среднего размера
        {0.1f, 3.0f}       // мелкие неровности
    };
}

std::vector<Vec2> Level::GetPoints(float carPositionX, float width, float precision) const
{
    std::vector<Vec2> result;

    float startX = carPositionX - width / 2.0f;
    float endX = carPositionX + width / 2.0f;

    std::mt19937 rng(sid); // генератор случайных чисел по сид
    std::uniform_real_distribution<float> noiseOffsetDist(0.0f, 1000.0f);

    // Случайный сдвиг для каждого шума
    std::vector<float> offsets;
    for (size_t i = 0; i < noiseComponents.size(); ++i)
        offsets.push_back(noiseOffsetDist(rng));

    for (float x = startX; x <= endX; x += precision)
    {
        float y = 0.0f;

        for (size_t i = 0; i < noiseComponents.size(); ++i)
        {
            float freq = noiseComponents[i].first;
            float amp = noiseComponents[i].second;
            float offset = offsets[i];

            y += std::sin((x + offset) * freq) * amp;
        }

        result.push_back(Vec2{x, y});
    }

    return result;
}