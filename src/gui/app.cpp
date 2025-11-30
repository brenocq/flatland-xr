// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"

#include <cmath>
#include <core/geometry.hpp>
#include <gui/app.hpp>
#include <gui/plot.hpp>
#include <map>

App::App() {}

void App::startup() {
    _camera.set_intrinsics(100, 60.0f * M_PI / 180.0f);
    _camera.set_noise_std(1.0f);
    _imu.set_acc_bias(Eigen::Vector2f(0.0f, 0.0f));
    _imu.set_gyr_bias(0.0f);
    _imu.set_acc_noise_std(Eigen::Vector2f(0.01f, 0.01f));
    _imu.set_gyr_noise_std(0.001f);
    load_world_preset(world::Preset::ASquaresHouse);
}

void App::shutdown() {}

void App::load_world_preset(world::Preset preset) {
    _current_preset = preset;
    _gt_pose_raw.clear();
    _gt_trajectory = core::Trajectory2D();
    _landmarks.clear();
    _walls.clear();
    _wall_raw_points.clear();

    if (preset == world::Preset::Custom) {
        return;
    }

    // Load preset data
    world::World world_data = world::load_preset(preset);
    _landmarks = world_data.landmarks;
    _walls = world_data.walls;

    // Generate dense trajectory from keypoints
    if (world_data.trajectory_keypoints.size() >= 2) {
        auto dense_poses = world::interpolate_trajectory(world_data.trajectory_keypoints, 60);
        _gt_trajectory.build(dense_poses);
    }
}

void App::update() {
    // Docking
    ImGuiID dockId = ImGui::GetID("##DockSpace");
    ImGui::DockSpaceOverViewport(dockId, ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockId, ImGuiCond_Appearing);
    if (ImGui::Begin("Flatland XR")) {
        render();
    }
    ImGui::End();

    // ImGui::ShowDemoWindow();
    // ImPlot::ShowDemoWindow();
}

void App::render() {
    bool should_simulate = _first_render;
    should_simulate |= render_config();
    should_simulate |= render_world_editor();
    if (should_simulate) {
        _first_render = false;
        simulate();
        estimate();
    }
    render_measurements();
    render_perception_output();
    render_error_metrics();
}

void App::simulate() {
    // Build trajectory from raw poses if available
    if (!_gt_pose_raw.empty()) {
        build_trajectory_from_raw_poses();
    }

    // Clear previous data
    _sim_result.clear();
    _est_poses.clear();
    _est_vel.clear();

    if (!_gt_trajectory.is_valid())
        return;

    // Run simulation
    _sim_result = simulation::run(_gt_trajectory, _landmarks, _walls, _camera, _imu, _sim_config);
}

void App::build_trajectory_from_raw_poses() {
    if (_gt_pose_raw.size() < 4) {
        _gt_trajectory = core::Trajectory2D();
        return;
    }

    // Smooth poses using a simple moving average filter
    std::vector<Eigen::Vector3f> smoothed_poses;
    smoothed_poses.reserve(_gt_pose_raw.size());

    const int window = 3; // Smoothing window half-size
    for (size_t i = 0; i < _gt_pose_raw.size(); i++) {
        float sum_x = 0, sum_y = 0;
        int count = 0;

        for (int j = -window; j <= window; j++) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(_gt_pose_raw.size())) {
                sum_x += _gt_pose_raw[idx].x();
                sum_y += _gt_pose_raw[idx].y();
                count++;
            }
        }

        float avg_x = sum_x / count;
        float avg_y = sum_y / count;

        // Compute orientation from smoothed positions
        float orientation = 0.0f;
        if (i > 0) {
            float dx = avg_x - smoothed_poses.back().x();
            float dy = avg_y - smoothed_poses.back().y();
            if (dx != 0 || dy != 0) {
                orientation = std::atan2(dy, dx);
            } else {
                orientation = smoothed_poses.back().z();
            }
        }

        smoothed_poses.push_back(Eigen::Vector3f(avg_x, avg_y, orientation));
    }

    // Subsample to reduce number of poses (keep roughly every Nth pose)
    std::vector<Eigen::Vector3f> subsampled;
    const size_t target_poses = std::min(smoothed_poses.size(), static_cast<size_t>(100));
    size_t step = std::max(static_cast<size_t>(1), smoothed_poses.size() / target_poses);

    for (size_t i = 0; i < smoothed_poses.size(); i += step) {
        subsampled.push_back(smoothed_poses[i]);
    }
    // Always include the last pose
    if (subsampled.back().x() != smoothed_poses.back().x() || subsampled.back().y() != smoothed_poses.back().y()) {
        subsampled.push_back(smoothed_poses.back());
    }

    // Recompute orientations after subsampling
    for (size_t i = 1; i < subsampled.size(); i++) {
        float dx = subsampled[i].x() - subsampled[i - 1].x();
        float dy = subsampled[i].y() - subsampled[i - 1].y();
        if (dx != 0 || dy != 0) {
            subsampled[i].z() = std::atan2(dy, dx);
        }
    }
    if (subsampled.size() > 1) {
        subsampled[0].z() = subsampled[1].z();
    }

    _gt_trajectory.build(subsampled);
}

void App::build_wall_from_raw_points() {
    if (_wall_raw_points.size() < 2) {
        return;
    }

    // Smooth wall points using moving average
    std::vector<Eigen::Vector2f> smoothed;
    smoothed.reserve(_wall_raw_points.size());

    const int window = 3;
    for (size_t i = 0; i < _wall_raw_points.size(); i++) {
        float sum_x = 0, sum_y = 0;
        int count = 0;

        for (int j = -window; j <= window; j++) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(_wall_raw_points.size())) {
                sum_x += _wall_raw_points[idx].x();
                sum_y += _wall_raw_points[idx].y();
                count++;
            }
        }
        smoothed.push_back(Eigen::Vector2f(sum_x / count, sum_y / count));
    }

    // Simplify wall by merging line segments with similar angles (Ramer-Douglas-Peucker style)
    std::vector<Eigen::Vector2f> simplified;
    simplified.push_back(smoothed.front());

    const float angle_threshold = 0.3f; // ~17 degrees - aggressive merging

    for (size_t i = 1; i < smoothed.size(); i++) {
        if (simplified.size() < 2) {
            simplified.push_back(smoothed[i]);
            continue;
        }

        // Get direction of current segment (from second-to-last to last simplified point)
        Eigen::Vector2f prev_dir = simplified.back() - simplified[simplified.size() - 2];
        prev_dir.normalize();

        // Get direction to new point
        Eigen::Vector2f new_dir = smoothed[i] - simplified.back();
        float new_len = new_dir.norm();
        if (new_len < 1e-6f)
            continue;
        new_dir.normalize();

        // Check angle between directions
        float dot = prev_dir.dot(new_dir);
        dot = std::clamp(dot, -1.0f, 1.0f);
        float angle = std::acos(dot);

        if (angle < angle_threshold) {
            // Similar direction - merge by moving last point
            simplified.back() = smoothed[i];
        } else {
            // Different direction - add new point
            simplified.push_back(smoothed[i]);
        }
    }

    // Only add wall if it has at least 2 points
    if (simplified.size() >= 2) {
        // Update or create current wall
        if (_walls.empty() || _wall_raw_points.size() == 1) {
            _walls.push_back({simplified});
        } else {
            _walls.back().points = simplified;
        }
    }
}

// Helper: check if two line segments intersect
static bool segments_intersect(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, const Eigen::Vector2f& p3, const Eigen::Vector2f& p4) {
    Eigen::Vector2f d1 = p2 - p1;
    Eigen::Vector2f d2 = p4 - p3;

    float cross = d1.x() * d2.y() - d1.y() * d2.x();
    if (std::abs(cross) < 1e-10f)
        return false; // Parallel

    Eigen::Vector2f d3 = p3 - p1;
    float t = (d3.x() * d2.y() - d3.y() * d2.x()) / cross;
    float u = (d3.x() * d1.y() - d3.y() * d1.x()) / cross;

    // Intersection within segments (with small epsilon to avoid endpoint issues)
    return t > 0.001f && t < 0.999f && u > 0.001f && u < 0.999f;
}

void App::estimate() {
    _est_poses.clear();
    _est_vel.clear();

    if (!_sim_result.is_valid())
        return;

    size_t num_poses = _sim_result.num_steps();
    _est_poses.reserve(num_poses);
    _est_vel.reserve(num_poses);

    // Initialize with ground truth initial state
    _est_poses.push_back(_sim_result.gt_poses[0]);
    _est_vel.push_back(_sim_result.gt_vel[0]);

    // Integrate IMU measurements (dt=1 since trajectory is parameterized by index)
    const float dt = 1.0f;

    for (size_t i = 1; i < num_poses; i++) {
        const sensors::IMUMeasurement& imu = _sim_result.imu_measurements[i - 1];
        Eigen::Vector3f prev_pose = _est_poses[i - 1];
        Eigen::Vector2f prev_vel = _est_vel[i - 1];
        float theta = prev_pose.z();

        // Remove bias from IMU measurements (we know the true bias for now)
        Eigen::Vector2f acc_body = imu.acc - _imu.acc_bias();
        float gyr = imu.gyr - _imu.gyr_bias();

        // Rotate acceleration from body frame to world frame
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);
        Eigen::Matrix2f R_bw; // Body to world rotation
        R_bw << cos_t, -sin_t, sin_t, cos_t;
        Eigen::Vector2f acc_world = R_bw * acc_body;

        // Add gravity back (accelerometer measures specific force = acc - gravity)
        // So world_acc = specific_force + gravity
        acc_world += _sim_config.gravity;

        // Integrate orientation: theta_new = theta + omega * dt
        float new_theta = theta + gyr * dt;

        // Integrate velocity: v_new = v + a * dt
        Eigen::Vector2f new_vel = prev_vel + acc_world * dt;

        // Integrate position: p_new = p + v * dt + 0.5 * a * dt^2
        Eigen::Vector2f new_pos = prev_pose.head<2>() + prev_vel * dt + 0.5f * acc_world * dt * dt;

        _est_poses.push_back(Eigen::Vector3f(new_pos.x(), new_pos.y(), new_theta));
        _est_vel.push_back(new_vel);
    }
}

bool App::render_config() {
    bool updated = false;
    if (ImGui::CollapsingHeader("Config", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent();
        ImGui::PushItemWidth(100);

        ImGui::Text("Simulation");
        if (ImGui::DragFloat("Time step (s)", &_dt, 0.01f, 0.01f, 1.0f)) {
            updated = true;
        }
        if (ImGui::DragFloat2("Gravity (m/s²)", _sim_config.gravity.data(), 0.1f, -20.0f, 20.0f)) {
            updated = true;
        }
        ImGui::Spacing();

        ImGui::Text("Camera Config");
        int cam_width = _camera.width();
        float cam_fov_deg = _camera.fov() * 180.0f / M_PI;
        float cam_noise_std = _camera.noise_std();
        if (ImGui::DragInt("Width (px)", &cam_width)) {
            _camera.set_width(cam_width);
            updated = true;
        }
        if (ImGui::DragFloat("FOV (deg)", &cam_fov_deg)) {
            _camera.set_fov(cam_fov_deg * M_PI / 180.0f);
            updated = true;
        }
        if (ImGui::DragFloat("Camera noise std", &cam_noise_std, 0.1f, 0.0f, 10.0f)) {
            _camera.set_noise_std(cam_noise_std);
            updated = true;
        }
        ImGui::Spacing();

        ImGui::Text("IMU Config");
        Eigen::Vector2f acc_bias = _imu.acc_bias();
        Eigen::Vector2f acc_noise_std = _imu.acc_noise_std();
        float gyr_bias = _imu.gyr_bias();
        float gyr_noise_std = _imu.gyr_noise_std();
        if (ImGui::DragFloat2("Accel bias", acc_bias.data(), 0.01f, -1.0f, 1.0f)) {
            _imu.set_acc_bias(acc_bias);
            updated = true;
        }
        if (ImGui::DragFloat2("Accel noise std", acc_noise_std.data(), 0.001f, 0.0f, 1.0f)) {
            _imu.set_acc_noise_std(acc_noise_std);
            updated = true;
        }
        if (ImGui::DragFloat("Gyro bias", &gyr_bias, 0.001f, -0.1f, 0.1f)) {
            _imu.set_gyr_bias(gyr_bias);
            updated = true;
        }
        if (ImGui::DragFloat("Gyro noise std", &gyr_noise_std, 0.0001f, 0.0f, 0.1f)) {
            _imu.set_gyr_noise_std(gyr_noise_std);
            updated = true;
        }

        ImGui::PopItemWidth();
        ImGui::Unindent();
    }
    return updated;
}

bool App::render_world_editor() {
    bool world_changed = false;
    static bool trajectory_drag_started = false;
    static ImVec2 trajectory_drag_start_pos;
    static bool landmark_click_started = false;
    static ImVec2 landmark_click_start_pos;
    static bool wall_drag_started = false;
    static ImVec2 wall_drag_start_pos;

    if (ImGui::CollapsingHeader("World Editor", ImGuiTreeNodeFlags_DefaultOpen)) {
        // World preset selector
        int current_preset_idx = static_cast<int>(_current_preset);
        ImGui::SetNextItemWidth(200);
        auto preset_getter = [](void*, int idx) { return world::preset_name(static_cast<world::Preset>(idx)); };
        if (ImGui::Combo("World Preset", &current_preset_idx, preset_getter, nullptr, static_cast<int>(world::Preset::COUNT))) {
            load_world_preset(static_cast<world::Preset>(current_preset_idx));
            world_changed = true;
        }
        ImGui::Separator();

        ImGui::TextWrapped("Ctrl+Click: Add new landmark | Ctrl+Drag: Draw trajectory");
        ImGui::SameLine();
        if (ImGui::Button("Clear trajectory")) {
            _gt_pose_raw.clear();
            _gt_trajectory = core::Trajectory2D();
            _current_preset = world::Preset::Custom;
            world_changed = true;
        }

        ImGui::TextWrapped("Right-click landmark: Delete");
        ImGui::SameLine();
        if (ImGui::Button("Clear all landmarks")) {
            _landmarks.clear();
            _current_preset = world::Preset::Custom;
            world_changed = true;
        }

        ImGui::TextWrapped("Shift+Drag: Draw wall");
        ImGui::SameLine();
        if (ImGui::Button("Clear all walls")) {
            _walls.clear();
            _wall_raw_points.clear();
            _current_preset = world::Preset::Custom;
            world_changed = true;
        }

        // Disable plot panning when Ctrl or Shift is held (we're drawing)
        ImPlotFlags plot_flags = ImPlotFlags_Equal;
        if (ImGui::GetIO().KeyCtrl || ImGui::GetIO().KeyShift) {
            plot_flags |= ImPlotFlags_NoInputs;
        }

        if (ImPlot::BeginPlot("##WorldEditor", ImVec2(-1, 400), plot_flags)) {
            ImPlot::SetupAxes("X (m)", "Y (m)");

            // Get plot limits and mouse position
            ImPlotPoint mouse = ImPlot::GetPlotMousePos();
            bool is_hovered = ImPlot::IsPlotHovered();

            // Handle Ctrl+click to add landmark or Ctrl+drag to draw trajectory
            if (is_hovered && ImGui::GetIO().KeyCtrl) {
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    trajectory_drag_started = true;
                    trajectory_drag_start_pos = ImGui::GetMousePos();
                    landmark_click_started = true;
                    landmark_click_start_pos = ImGui::GetMousePos();
                }

                if (trajectory_drag_started && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    ImVec2 current_pos = ImGui::GetMousePos();
                    float drag_dist = std::sqrt(std::pow(current_pos.x - trajectory_drag_start_pos.x, 2) +
                                                std::pow(current_pos.y - trajectory_drag_start_pos.y, 2));

                    // If dragged more than threshold, switch to trajectory drawing mode
                    if (drag_dist > 5.0f) {
                        // Clear trajectory on first drag movement
                        if (landmark_click_started) {
                            _gt_pose_raw.clear();
                            _gt_trajectory = core::Trajectory2D();
                            landmark_click_started = false;
                            _current_preset = world::Preset::Custom;
                        }

                        bool should_add = true;
                        if (!_gt_pose_raw.empty()) {
                            ImPlotPoint last_plot(_gt_pose_raw.back().x(), _gt_pose_raw.back().y());
                            ImVec2 last_px = ImPlot::PlotToPixels(last_plot);
                            ImVec2 mouse_px = ImPlot::PlotToPixels(mouse);
                            float dist = std::sqrt(std::pow(mouse_px.x - last_px.x, 2) + std::pow(mouse_px.y - last_px.y, 2));
                            should_add = dist > 1.0f;
                        }
                        if (should_add) {
                            // Compute orientation from previous pose
                            float orientation = 0.0f;
                            if (!_gt_pose_raw.empty()) {
                                Eigen::Vector2f prev(_gt_pose_raw.back().x(), _gt_pose_raw.back().y());
                                Eigen::Vector2f curr(mouse.x, mouse.y);
                                Eigen::Vector2f dir = curr - prev;
                                orientation = std::atan2(dir.y(), dir.x());
                            }
                            _gt_pose_raw.push_back(Eigen::Vector3f(mouse.x, mouse.y, orientation));
                            // Rebuild trajectory while drawing for live preview
                            build_trajectory_from_raw_poses();
                        }
                    }
                }

                // On release, if we didn't drag much, add a landmark
                if (landmark_click_started && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    ImVec2 current_pos = ImGui::GetMousePos();
                    float dist =
                        std::sqrt(std::pow(current_pos.x - landmark_click_start_pos.x, 2) + std::pow(current_pos.y - landmark_click_start_pos.y, 2));
                    if (dist < 5.0f) {
                        _landmarks.push_back(Eigen::Vector2f(mouse.x, mouse.y));
                        _current_preset = world::Preset::Custom;
                        world_changed = true;
                    }
                }
            }

            // Reset tracking on mouse release and mark world as changed if we drew something
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                if (trajectory_drag_started && !landmark_click_started && !_gt_pose_raw.empty()) {
                    world_changed = true; // Trajectory was drawn
                }
                trajectory_drag_started = false;
                landmark_click_started = false;
                if (wall_drag_started) {
                    wall_drag_started = false;
                    _wall_raw_points.clear(); // Clear raw points after wall is finalized
                    world_changed = true;     // Wall was drawn
                }
            }

            // Handle Shift+drag to draw walls
            if (is_hovered && ImGui::GetIO().KeyShift && !ImGui::GetIO().KeyCtrl) {
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    wall_drag_started = true;
                    wall_drag_start_pos = ImGui::GetMousePos();
                    _wall_raw_points.clear();
                }

                if (wall_drag_started && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    ImVec2 current_pos = ImGui::GetMousePos();
                    float drag_dist =
                        std::sqrt(std::pow(current_pos.x - wall_drag_start_pos.x, 2) + std::pow(current_pos.y - wall_drag_start_pos.y, 2));

                    if (drag_dist > 3.0f) {
                        // Start a new wall if this is the first point
                        if (_wall_raw_points.empty()) {
                            _walls.push_back(core::Wall{});
                            _current_preset = world::Preset::Custom;
                        }

                        bool should_add = true;
                        if (!_wall_raw_points.empty()) {
                            ImPlotPoint last_plot(_wall_raw_points.back().x(), _wall_raw_points.back().y());
                            ImVec2 last_px = ImPlot::PlotToPixels(last_plot);
                            ImVec2 mouse_px_check = ImPlot::PlotToPixels(mouse);
                            float dist = std::sqrt(std::pow(mouse_px_check.x - last_px.x, 2) + std::pow(mouse_px_check.y - last_px.y, 2));
                            should_add = dist > 1.0f;
                        }
                        if (should_add) {
                            _wall_raw_points.push_back(Eigen::Vector2f(mouse.x, mouse.y));
                            build_wall_from_raw_points();
                        }
                    }
                }
            }

            // Find closest point (landmark or GT) for tooltip
            ImVec2 mouse_px = ImGui::GetMousePos();
            float closest_dist = 5.0f;
            int closest_landmark = -1;
            int closest_gt = -1;

            for (size_t i = 0; i < _landmarks.size(); i++) {
                ImVec2 lm_px = ImPlot::PlotToPixels(ImPlotPoint(_landmarks[i].x(), _landmarks[i].y()));
                float dist = std::sqrt(std::pow(mouse_px.x - lm_px.x, 2) + std::pow(mouse_px.y - lm_px.y, 2));
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_landmark = static_cast<int>(i);
                    closest_gt = -1;
                }
            }

            // Check proximity to trajectory poses
            if (_gt_trajectory.is_valid()) {
                for (size_t i = 0; i < _gt_trajectory.num_poses(); i++) {
                    Eigen::Vector2f pos = _gt_trajectory.position(static_cast<float>(i));
                    ImVec2 gt_px = ImPlot::PlotToPixels(ImPlotPoint(pos.x(), pos.y()));
                    float dist = std::sqrt(std::pow(mouse_px.x - gt_px.x, 2) + std::pow(mouse_px.y - gt_px.y, 2));
                    if (dist < closest_dist) {
                        closest_dist = dist;
                        closest_gt = static_cast<int>(i);
                        closest_landmark = -1;
                    }
                }
            }

            // Render trajectory
            if (_gt_trajectory.is_valid()) {
                plot_2d_trajectory("Trajectory", _gt_trajectory, Color::Green());
            }

            // While drawing, show camera at last pose; otherwise show tooltip for closest point
            bool is_drawing = trajectory_drag_started && !landmark_click_started;
            if (is_drawing && _gt_trajectory.is_valid()) {
                // Show camera at last pose while drawing
                float last_t = _gt_trajectory.max_t();
                Eigen::Vector3f pose = _gt_trajectory.pose_vector(last_t);
                Eigen::Vector2f pos(pose.x(), pose.y());
                // Filter landmarks by wall occlusion, keeping track of original indices
                std::vector<sensors::LandmarkObservation> observations;
                for (size_t i = 0; i < _landmarks.size(); i++) {
                    if (!simulation::is_landmark_occluded(pos, _landmarks[i], _walls)) {
                        auto u = _camera.project(pose, _landmarks[i]);
                        if (u.has_value()) {
                            observations.push_back({u.value(), i});
                        }
                    }
                }
                plot_2d_camera_frustum("##DrawingCamera", pos, pose.z(), _camera.fov(), 1.0f, Color::Blue());
                plot_2d_camera_rays("##DrawingRays", pos, _landmarks, observations, 1.0f);
                plot_2d_camera_observations("##DrawingObs", pos, pose.z(), _camera, observations);
            } else {
                // Show tooltip for closest point and camera preview for poses
                if (closest_landmark >= 0) {
                    // Count observations for this landmark (considering wall occlusion)
                    int obs_count = 0;
                    if (_gt_trajectory.is_valid()) {
                        int num_poses = static_cast<int>(_gt_trajectory.max_t()) + 1;
                        for (int t = 0; t < num_poses; t++) {
                            Eigen::Vector3f pose = _gt_trajectory.pose_vector(static_cast<float>(t));
                            Eigen::Vector2f pos(pose.x(), pose.y());
                            // Check wall occlusion before projecting
                            if (simulation::is_landmark_occluded(pos, _landmarks[closest_landmark], _walls))
                                continue;
                            auto u = _camera.project(pose, _landmarks[closest_landmark]);
                            if (u.has_value()) {
                                obs_count++;
                                // Only show observation for the hovered landmark
                                std::vector<sensors::LandmarkObservation> single_obs = {{u.value(), static_cast<size_t>(closest_landmark)}};
                                plot_2d_camera_frustum("##LandmarkHoverCamera", pos, pose.z(), _camera.fov(), 1.0f, Color::Blue());
                                plot_2d_camera_rays("##LandmarkHoverRays", pos, _landmarks, single_obs, 1.0f);
                                plot_2d_camera_observations("##LandmarkHoverObs", pos, pose.z(), _camera, single_obs);
                            }
                        }
                    }
                    ImGui::SetTooltip("Landmark %d\nPos: (%.2f, %.2f)\nObservations: %d", closest_landmark, _landmarks[closest_landmark].x(),
                                      _landmarks[closest_landmark].y(), obs_count);
                } else if (closest_gt >= 0 && _gt_trajectory.is_valid()) {
                    Eigen::Vector3f pose = _gt_trajectory.pose_vector(static_cast<float>(closest_gt));
                    // Draw camera frustum with projected landmarks (filtered by walls)
                    Eigen::Vector2f pos(pose.x(), pose.y());
                    std::vector<sensors::LandmarkObservation> observations;
                    for (size_t i = 0; i < _landmarks.size(); i++) {
                        if (!simulation::is_landmark_occluded(pos, _landmarks[i], _walls)) {
                            auto u = _camera.project(pose, _landmarks[i]);
                            if (u.has_value()) {
                                observations.push_back({u.value(), i});
                            }
                        }
                    }
                    ImGui::SetTooltip("GT Pose %d\nPos: (%.2f, %.2f)\nOrientation: %.2f°\nObservations: %zu", closest_gt, pose.x(), pose.y(),
                                      pose.z() * 180.0f / M_PI, observations.size());
                    plot_2d_camera_frustum("##HoverCamera", pos, pose.z(), _camera.fov(), 1.0f, Color::Blue());
                    plot_2d_camera_rays("##HoverRays", pos, _landmarks, observations, 1.0f);
                    plot_2d_camera_observations("##HoverObs", pos, pose.z(), _camera, observations);
                }
            }

            // Render landmarks as draggable points with context menu
            int landmark_to_delete = -1;
            for (size_t i = 0; i < _landmarks.size(); i++) {
                double x = _landmarks[i].x();
                double y = _landmarks[i].y();
                Color lm_color = Color::Random(i);
                if (ImPlot::DragPoint(static_cast<int>(i), &x, &y, ImVec4(lm_color), 4.0f)) {
                    _landmarks[i] = Eigen::Vector2f(x, y);
                    _current_preset = world::Preset::Custom;
                    world_changed = true;
                }

                // Check if mouse is hovering this landmark for context menu
                ImVec2 lm_px = ImPlot::PlotToPixels(ImPlotPoint(x, y));
                float dist = std::sqrt(std::pow(mouse_px.x - lm_px.x, 2) + std::pow(mouse_px.y - lm_px.y, 2));
                if (dist < 10.0f) {
                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                        ImGui::OpenPopup(("LandmarkContext" + std::to_string(i)).c_str());
                    }
                }

                if (ImGui::BeginPopup(("LandmarkContext" + std::to_string(i)).c_str())) {
                    ImGui::Text("Landmark %zu", i);
                    ImGui::Separator();
                    if (ImGui::MenuItem("Delete")) {
                        landmark_to_delete = static_cast<int>(i);
                    }
                    ImGui::EndPopup();
                }
            }
            if (landmark_to_delete >= 0) {
                _landmarks.erase(_landmarks.begin() + landmark_to_delete);
                _current_preset = world::Preset::Custom;
                world_changed = true;
            }

            // Render walls (plain white)
            int wall_to_delete = -1;
            for (size_t w = 0; w < _walls.size(); w++) {
                const auto& wall = _walls[w];
                if (wall.points.size() >= 2) {
                    plot_2d_line("##Wall" + std::to_string(w), wall.points, Color::White(), 2.0f);

                    // Check if mouse is near any segment of this wall for context menu
                    for (size_t i = 0; i + 1 < wall.points.size(); i++) {
                        ImVec2 p1_px = ImPlot::PlotToPixels(ImPlotPoint(wall.points[i].x(), wall.points[i].y()));
                        ImVec2 p2_px = ImPlot::PlotToPixels(ImPlotPoint(wall.points[i + 1].x(), wall.points[i + 1].y()));

                        // Distance from mouse to line segment
                        float dx = p2_px.x - p1_px.x;
                        float dy = p2_px.y - p1_px.y;
                        float len_sq = dx * dx + dy * dy;
                        float t = 0.0f;
                        if (len_sq > 0) {
                            t = std::clamp(((mouse_px.x - p1_px.x) * dx + (mouse_px.y - p1_px.y) * dy) / len_sq, 0.0f, 1.0f);
                        }
                        float closest_x = p1_px.x + t * dx;
                        float closest_y = p1_px.y + t * dy;
                        float dist = std::sqrt(std::pow(mouse_px.x - closest_x, 2) + std::pow(mouse_px.y - closest_y, 2));

                        if (dist < 10.0f && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                            ImGui::OpenPopup(("WallContext" + std::to_string(w)).c_str());
                            break;
                        }
                    }
                }

                if (ImGui::BeginPopup(("WallContext" + std::to_string(w)).c_str())) {
                    ImGui::Text("Wall %zu (%zu segments)", w, wall.points.size() > 0 ? wall.points.size() - 1 : 0);
                    ImGui::Separator();
                    if (ImGui::MenuItem("Delete")) {
                        wall_to_delete = static_cast<int>(w);
                    }
                    ImGui::EndPopup();
                }
            }
            if (wall_to_delete >= 0) {
                _walls.erase(_walls.begin() + wall_to_delete);
                _current_preset = world::Preset::Custom;
                world_changed = true;
            }

            ImPlot::EndPlot();
        }
    }
    return world_changed;
}

void App::render_measurements() {
    if (ImGui::CollapsingHeader("Sensor Measurements")) {
        if (!_sim_result.is_valid()) {
            ImGui::Text("No measurements available. Draw a trajectory first.");
            return;
        }

        size_t num_steps = _sim_result.num_steps();

        // Prepare time index axis
        std::vector<float> time_axis(num_steps);
        for (size_t i = 0; i < num_steps; i++) {
            time_axis[i] = static_cast<float>(i);
        }

        // IMU Accelerometer plot
        if (ImPlot::BeginPlot("IMU Accelerometer", ImVec2(-1, 200))) {
            ImPlot::SetupAxes("Time Index", "Acceleration (m/s²)");

            // Extract data
            std::vector<float> gt_acc_x(num_steps), gt_acc_y(num_steps);
            std::vector<float> meas_acc_x(num_steps), meas_acc_y(num_steps);
            for (size_t i = 0; i < num_steps; i++) {
                gt_acc_x[i] = _sim_result.gt_imu[i].acc.x();
                gt_acc_y[i] = _sim_result.gt_imu[i].acc.y();
                meas_acc_x[i] = _sim_result.imu_measurements[i].acc.x();
                meas_acc_y[i] = _sim_result.imu_measurements[i].acc.y();
            }

            // Colors: measurement colors and faded ground truth
            Color color_x = Color::Red();
            Color color_y = Color::Blue();
            Color gt_color_x = Color(0.25f * color_x.r() + 0.75f, 0.25f * color_x.g() + 0.75f, 0.25f * color_x.b() + 0.75f);
            Color gt_color_y = Color(0.25f * color_y.r() + 0.75f, 0.25f * color_y.g() + 0.75f, 0.25f * color_y.b() + 0.75f);

            // Plot ground truth (faded)
            ImPlot::SetNextLineStyle(ImVec4(gt_color_x), 1.0f);
            ImPlot::PlotLine("GT Acc X", time_axis.data(), gt_acc_x.data(), static_cast<int>(num_steps));
            ImPlot::SetNextLineStyle(ImVec4(gt_color_y), 1.0f);
            ImPlot::PlotLine("GT Acc Y", time_axis.data(), gt_acc_y.data(), static_cast<int>(num_steps));

            // Plot measurements
            ImPlot::SetNextLineStyle(ImVec4(color_x), 2.0f);
            ImPlot::PlotLine("Acc X", time_axis.data(), meas_acc_x.data(), static_cast<int>(num_steps));
            ImPlot::SetNextLineStyle(ImVec4(color_y), 2.0f);
            ImPlot::PlotLine("Acc Y", time_axis.data(), meas_acc_y.data(), static_cast<int>(num_steps));

            ImPlot::EndPlot();
        }

        // IMU Gyroscope plot
        if (ImPlot::BeginPlot("IMU Gyroscope", ImVec2(-1, 200))) {
            ImPlot::SetupAxes("Time Index", "Angular velocity (rad/s)");

            // Extract data
            std::vector<float> gt_gyr(num_steps), meas_gyr(num_steps);
            for (size_t i = 0; i < num_steps; i++) {
                gt_gyr[i] = _sim_result.gt_imu[i].gyr;
                meas_gyr[i] = _sim_result.imu_measurements[i].gyr;
            }

            // Colors
            Color color_gyr = Color::Green();
            Color gt_color_gyr = Color(0.25f * color_gyr.r() + 0.75f, 0.25f * color_gyr.g() + 0.75f, 0.25f * color_gyr.b() + 0.75f);

            // Plot ground truth (faded)
            ImPlot::SetNextLineStyle(ImVec4(gt_color_gyr), 1.0f);
            ImPlot::PlotLine("GT Gyro", time_axis.data(), gt_gyr.data(), static_cast<int>(num_steps));

            // Plot measurements
            ImPlot::SetNextLineStyle(ImVec4(color_gyr), 2.0f);
            ImPlot::PlotLine("Gyro", time_axis.data(), meas_gyr.data(), static_cast<int>(num_steps));

            ImPlot::EndPlot();
        }

        // Camera measurements plot (Time Index on X axis, Image u on Y axis)
        if (ImPlot::BeginPlot("Camera Observations", ImVec2(-1, 300))) {
            ImPlot::SetupAxes("Time Index", "Image u (px)");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, static_cast<double>(num_steps));
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, _camera.width());

            // For each landmark, collect observations across time
            // Store as (time_index, u) pairs, then split into consecutive segments
            std::map<size_t, std::vector<std::pair<int, float>>> gt_tracks;
            std::map<size_t, std::vector<std::pair<int, float>>> meas_tracks;

            for (size_t t = 0; t < num_steps; t++) {
                int time_idx = static_cast<int>(t);
                for (const auto& obs : _sim_result.gt_cam[t]) {
                    gt_tracks[obs.landmark_id].push_back({time_idx, obs.u});
                }
                for (const auto& obs : _sim_result.cam_measurements[t]) {
                    meas_tracks[obs.landmark_id].push_back({time_idx, obs.u});
                }
            }

            // Helper to split track into consecutive segments
            auto split_into_segments = [](const std::vector<std::pair<int, float>>& track) {
                std::vector<std::vector<std::pair<int, float>>> segments;
                if (track.empty())
                    return segments;

                std::vector<std::pair<int, float>> current_segment;
                current_segment.push_back(track[0]);

                for (size_t i = 1; i < track.size(); i++) {
                    if (track[i].first == track[i - 1].first + 1) {
                        // Consecutive, add to current segment
                        current_segment.push_back(track[i]);
                    } else {
                        // Gap detected, start new segment
                        if (!current_segment.empty()) {
                            segments.push_back(current_segment);
                        }
                        current_segment.clear();
                        current_segment.push_back(track[i]);
                    }
                }
                if (!current_segment.empty()) {
                    segments.push_back(current_segment);
                }
                return segments;
            };

            // Draw tracks for each landmark
            for (const auto& [lm_id, track] : meas_tracks) {
                if (track.empty())
                    continue;

                Color lm_color = Color::Random(lm_id);
                Color gt_lm_color = Color(0.25f * lm_color.r() + 0.75f, 0.25f * lm_color.g() + 0.75f, 0.25f * lm_color.b() + 0.75f);
                std::string label = "LM " + std::to_string(lm_id);

                // Get ground truth track for this landmark
                const auto& gt_track = gt_tracks[lm_id];
                auto gt_segments = split_into_segments(gt_track);
                auto meas_segments = split_into_segments(track);

                // Draw ground truth segments (faded lines)
                for (size_t seg_idx = 0; seg_idx < gt_segments.size(); seg_idx++) {
                    const auto& seg = gt_segments[seg_idx];
                    if (seg.size() >= 2) {
                        std::vector<float> seg_t, seg_u;
                        for (const auto& [t, u] : seg) {
                            seg_t.push_back(static_cast<float>(t));
                            seg_u.push_back(u);
                        }
                        ImPlot::SetNextLineStyle(ImVec4(gt_lm_color), 1.0f);
                        ImPlot::PlotLine(label.c_str(), seg_t.data(), seg_u.data(), static_cast<int>(seg_t.size()));
                    }
                }

                // Draw ground truth scatter points
                if (!gt_track.empty()) {
                    std::vector<float> gt_t, gt_u;
                    for (const auto& [t, u] : gt_track) {
                        gt_t.push_back(static_cast<float>(t));
                        gt_u.push_back(u);
                    }
                    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f, ImVec4(gt_lm_color), IMPLOT_AUTO, ImVec4(gt_lm_color));
                    ImPlot::PlotScatter(label.c_str(), gt_t.data(), gt_u.data(), static_cast<int>(gt_t.size()));
                }

                // Draw measurement segments (lines)
                for (size_t seg_idx = 0; seg_idx < meas_segments.size(); seg_idx++) {
                    const auto& seg = meas_segments[seg_idx];
                    if (seg.size() >= 2) {
                        std::vector<float> seg_t, seg_u;
                        for (const auto& [t, u] : seg) {
                            seg_t.push_back(static_cast<float>(t));
                            seg_u.push_back(u);
                        }
                        ImPlot::SetNextLineStyle(ImVec4(lm_color), 2.0f);
                        // Only use visible label for first segment
                        if (seg_idx == 0) {
                            ImPlot::PlotLine(label.c_str(), seg_t.data(), seg_u.data(), static_cast<int>(seg_t.size()));
                        } else {
                            ImPlot::PlotLine(label.c_str(), seg_t.data(), seg_u.data(), static_cast<int>(seg_t.size()));
                        }
                    }
                }

                // Draw measurement scatter points
                if (!track.empty()) {
                    std::vector<float> meas_t, meas_u;
                    for (const auto& [t, u] : track) {
                        meas_t.push_back(static_cast<float>(t));
                        meas_u.push_back(u);
                    }
                    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4.0f, ImVec4(lm_color), IMPLOT_AUTO, ImVec4(lm_color));
                    ImPlot::PlotScatter(label.c_str(), meas_t.data(), meas_u.data(), static_cast<int>(meas_t.size()));
                }
            }

            ImPlot::EndPlot();
        }
    }
}

void App::render_perception_output() {
    if (ImGui::CollapsingHeader("Perception Output", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (_est_poses.empty() || !_sim_result.is_valid()) {
            ImGui::Text("No estimation data available.");
            return;
        }

        size_t num_poses = _est_poses.size();

        // Prepare time index axis
        std::vector<float> time_axis(num_poses);
        for (size_t i = 0; i < num_poses; i++) {
            time_axis[i] = static_cast<float>(i);
        }

        // 2D trajectory plot
        if (ImPlot::BeginPlot("Trajectory", ImVec2(-1, 300), ImPlotFlags_Equal)) {
            if (_gt_trajectory.is_valid()) {
                plot_2d_trajectory("Ground-truth", _gt_trajectory, Color::Green());
            }
            std::vector<Eigen::Vector2f> est_positions;
            est_positions.reserve(num_poses);
            for (const auto& pose : _est_poses) {
                est_positions.push_back(pose.head<2>());
            }
            plot_2d_path("Estimated", est_positions, Color::Red());
            ImPlot::EndPlot();
        }

        // Position X and Y plots
        if (ImPlot::BeginPlot("Position", ImVec2(-1, 200))) {
            ImPlot::SetupAxes("Time Index", "Position (m)");

            std::vector<float> gt_x(num_poses), gt_y(num_poses);
            std::vector<float> est_x(num_poses), est_y(num_poses);
            for (size_t i = 0; i < num_poses; i++) {
                gt_x[i] = _sim_result.gt_poses[i].x();
                gt_y[i] = _sim_result.gt_poses[i].y();
                est_x[i] = _est_poses[i].x();
                est_y[i] = _est_poses[i].y();
            }

            // Ground truth (faded)
            Color gt_color_x = Color(0.25f * 1.0f + 0.75f, 0.25f * 0.0f + 0.75f, 0.25f * 0.0f + 0.75f);
            Color gt_color_y = Color(0.25f * 0.0f + 0.75f, 0.25f * 0.0f + 0.75f, 0.25f * 1.0f + 0.75f);
            ImPlot::SetNextLineStyle(ImVec4(gt_color_x), 1.0f);
            ImPlot::PlotLine("GT X", time_axis.data(), gt_x.data(), static_cast<int>(num_poses));
            ImPlot::SetNextLineStyle(ImVec4(gt_color_y), 1.0f);
            ImPlot::PlotLine("GT Y", time_axis.data(), gt_y.data(), static_cast<int>(num_poses));

            // Estimated
            ImPlot::SetNextLineStyle(ImVec4(Color::Red()), 2.0f);
            ImPlot::PlotLine("Est X", time_axis.data(), est_x.data(), static_cast<int>(num_poses));
            ImPlot::SetNextLineStyle(ImVec4(Color::Blue()), 2.0f);
            ImPlot::PlotLine("Est Y", time_axis.data(), est_y.data(), static_cast<int>(num_poses));

            ImPlot::EndPlot();
        }

        // Orientation plot
        if (ImPlot::BeginPlot("Orientation", ImVec2(-1, 200))) {
            ImPlot::SetupAxes("Time Index", "Orientation (rad)");

            std::vector<float> gt_theta(num_poses), est_theta(num_poses);
            for (size_t i = 0; i < num_poses; i++) {
                gt_theta[i] = _sim_result.gt_poses[i].z();
                est_theta[i] = _est_poses[i].z();
            }

            Color gt_color = Color(0.25f * 0.0f + 0.75f, 0.25f * 0.5f + 0.75f, 0.25f * 0.0f + 0.75f);
            ImPlot::SetNextLineStyle(ImVec4(gt_color), 1.0f);
            ImPlot::PlotLine("GT Theta", time_axis.data(), gt_theta.data(), static_cast<int>(num_poses));

            ImPlot::SetNextLineStyle(ImVec4(Color::Green()), 2.0f);
            ImPlot::PlotLine("Est Theta", time_axis.data(), est_theta.data(), static_cast<int>(num_poses));

            ImPlot::EndPlot();
        }

        // Velocity plot
        if (ImPlot::BeginPlot("Velocity", ImVec2(-1, 200))) {
            ImPlot::SetupAxes("Time Index", "Velocity (m/idx)");

            std::vector<float> gt_vx(num_poses), gt_vy(num_poses);
            std::vector<float> est_vx(num_poses), est_vy(num_poses);
            for (size_t i = 0; i < num_poses; i++) {
                gt_vx[i] = _sim_result.gt_vel[i].x();
                gt_vy[i] = _sim_result.gt_vel[i].y();
                est_vx[i] = _est_vel[i].x();
                est_vy[i] = _est_vel[i].y();
            }

            // Ground truth (faded)
            Color gt_color_vx = Color(0.25f * 1.0f + 0.75f, 0.25f * 0.0f + 0.75f, 0.25f * 0.0f + 0.75f);
            Color gt_color_vy = Color(0.25f * 0.0f + 0.75f, 0.25f * 0.0f + 0.75f, 0.25f * 1.0f + 0.75f);
            ImPlot::SetNextLineStyle(ImVec4(gt_color_vx), 1.0f);
            ImPlot::PlotLine("GT Vx", time_axis.data(), gt_vx.data(), static_cast<int>(num_poses));
            ImPlot::SetNextLineStyle(ImVec4(gt_color_vy), 1.0f);
            ImPlot::PlotLine("GT Vy", time_axis.data(), gt_vy.data(), static_cast<int>(num_poses));

            // Estimated
            ImPlot::SetNextLineStyle(ImVec4(Color::Red()), 2.0f);
            ImPlot::PlotLine("Est Vx", time_axis.data(), est_vx.data(), static_cast<int>(num_poses));
            ImPlot::SetNextLineStyle(ImVec4(Color::Blue()), 2.0f);
            ImPlot::PlotLine("Est Vy", time_axis.data(), est_vy.data(), static_cast<int>(num_poses));

            ImPlot::EndPlot();
        }
    }
}

void App::render_error_metrics() {
    if (ImGui::CollapsingHeader("Error Metrics")) {
        ImGui::Text("TODO");
    }
}
