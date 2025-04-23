#pragma once
#include "tick_system.hpp"
#include "imgui.h"

inline void TickSystemImGui(TickSystem &tickSystem)
{
    ImGui::Begin("Tick System");

    ImGui::Text("Tick Rate: %.2f Hz", 1.0f / tickSystem.GetFixedDt());
    float tickRate = tickSystem.GetTickRate();
    if (ImGui::SliderFloat("Tick rate", &tickRate, 0.1f, 120.0f, "%.2fx"))
        tickSystem.SetTickRate(tickRate);

    if (ImGui::Button("Step"))
        tickSystem.StepOnce();
    ImGui::SameLine();
    if (ImGui::Button(tickSystem.IsPaused() ? "Play" : "Pause"))
        tickSystem.SetIsPause(!tickSystem.IsPaused());

    float timeScale = tickSystem.GetTimeScale();
    if (ImGui::SliderFloat("Time Scale", &timeScale, 0.1f, 100.0f, "%.2fx"))
        tickSystem.SetTimeScale(timeScale);

    ImGui::End();
}
