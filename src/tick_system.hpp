#pragma once
#include <algorithm>

class TickSystem
{
public:
    TickSystem(float tickRate = 60.0f)
        : mFixedDt(1.0f / tickRate), mAccumulator(0.0f), mPaused(false), mStepOnce(false), mTimeScale(1.f) {}

    void Update(float realDt)
    {
        if (!mPaused)
            mAccumulator += realDt;
    }

    bool ShouldStep() const
    {
        return mAccumulator >= mFixedDt || mStepOnce;
    }

    bool Step()
    {
        if (ShouldStep())
        {
            mAccumulator -= mFixedDt;
            if (mAccumulator < 0)
                mAccumulator = 0;

            mStepOnce = false;
            return true;
        }
        return false;
    }

    float GetFixedDt() const { return mFixedDt * mTimeScale; }

    float GetTickRate() { return 1.f / mFixedDt; }
    void SetTickRate(float tickRate) { mFixedDt = 1.f / tickRate; }

    float GetTimeScale() const { return mTimeScale; }
    void SetTimeScale(float timeScale) { mTimeScale = timeScale; }

    void SetIsPause(bool value) { mPaused = value; }
    bool IsPaused() const { return mPaused; }

    void StepOnce() { mStepOnce = true; }

private:
    float mFixedDt;
    float mAccumulator;
    float mTimeScale;
    bool mPaused;
    bool mStepOnce;
};
