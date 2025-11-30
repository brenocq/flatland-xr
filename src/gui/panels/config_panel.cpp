// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "config_panel.hpp"
#include "imgui.h"
#include <cmath>

namespace gui {

bool ConfigPanel::render(float& dt, simulation::SimulationConfig& sim_config, sensors::Camera2D& camera, sensors::IMU2D& imu) {
    bool updated = false;
    if (ImGui::CollapsingHeader("Config", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent();
        ImGui::PushItemWidth(100);

        ImGui::Text("Simulation");
        if (ImGui::DragFloat("Time step (s)", &dt, 0.01f, 0.01f, 1.0f)) {
            updated = true;
        }
        if (ImGui::DragFloat2("Gravity (m/s²)", sim_config.gravity.data(), 0.1f, -20.0f, 20.0f)) {
            updated = true;
        }
        ImGui::Spacing();

        ImGui::Text("Camera Config");
        int cam_width = camera.width();
        float cam_fov_deg = camera.fov() * 180.0f / M_PI;
        float cam_noise_std = camera.noise_std();
        if (ImGui::DragInt("Width (px)", &cam_width)) {
            camera.set_width(cam_width);
            updated = true;
        }
        if (ImGui::DragFloat("FOV (deg)", &cam_fov_deg)) {
            camera.set_fov(cam_fov_deg * M_PI / 180.0f);
            updated = true;
        }
        if (ImGui::DragFloat("Camera noise std", &cam_noise_std, 0.1f, 0.0f, 10.0f)) {
            camera.set_noise_std(cam_noise_std);
            updated = true;
        }
        ImGui::Spacing();

        ImGui::Text("IMU Config");
        Eigen::Vector2f acc_bias = imu.acc_bias();
        Eigen::Vector2f acc_noise_std = imu.acc_noise_std();
        float gyr_bias = imu.gyr_bias();
        float gyr_noise_std = imu.gyr_noise_std();
        if (ImGui::DragFloat2("Accel bias", acc_bias.data(), 0.01f, -1.0f, 1.0f)) {
            imu.set_acc_bias(acc_bias);
            updated = true;
        }
        if (ImGui::DragFloat2("Accel noise std", acc_noise_std.data(), 0.001f, 0.0f, 1.0f)) {
            imu.set_acc_noise_std(acc_noise_std);
            updated = true;
        }
        if (ImGui::DragFloat("Gyro bias", &gyr_bias, 0.001f, -0.1f, 0.1f)) {
            imu.set_gyr_bias(gyr_bias);
            updated = true;
        }
        if (ImGui::DragFloat("Gyro noise std", &gyr_noise_std, 0.0001f, 0.0f, 0.1f)) {
            imu.set_gyr_noise_std(gyr_noise_std);
            updated = true;
        }

        ImGui::PopItemWidth();
        ImGui::Unindent();
    }
    return updated;
}

} // namespace gui
