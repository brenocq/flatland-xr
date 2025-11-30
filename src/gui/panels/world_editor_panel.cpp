// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "imgui.h"
#include "implot.h"
#include <core/math.hpp>
#include <gui/color.hpp>
#include <gui/panels/world_editor_panel.hpp>
#include <gui/plot.hpp>
#include <gui/ui_state.hpp>
#include <simulation/simulation.hpp>

namespace gui {

namespace {

/// Smooth raw poses and build trajectory
void build_trajectory_from_raw_poses(const std::vector<Eigen::Vector3f>& gt_pose_raw, core::Trajectory2D& gt_trajectory) {
    if (gt_pose_raw.size() < 4) {
        gt_trajectory = core::Trajectory2D();
        return;
    }

    // Smooth poses using a simple moving average filter
    std::vector<Eigen::Vector3f> smoothed_poses;
    smoothed_poses.reserve(gt_pose_raw.size());

    const int window = 3; // Smoothing window half-size
    for (size_t i = 0; i < gt_pose_raw.size(); i++) {
        float sum_x = 0, sum_y = 0;
        int count = 0;

        for (int j = -window; j <= window; j++) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(gt_pose_raw.size())) {
                sum_x += gt_pose_raw[idx].x();
                sum_y += gt_pose_raw[idx].y();
                count++;
            }
        }

        float avg_x = sum_x / static_cast<float>(count);
        float avg_y = sum_y / static_cast<float>(count);

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

        smoothed_poses.emplace_back(avg_x, avg_y, orientation);
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

    gt_trajectory.build(subsampled);
}

/// Smooth raw wall points and build simplified wall
void build_wall_from_raw_points(std::vector<core::Wall>& walls, const std::vector<Eigen::Vector2f>& wall_raw_points) {
    if (wall_raw_points.size() < 2) {
        return;
    }

    // Smooth wall points using moving average
    std::vector<Eigen::Vector2f> smoothed;
    smoothed.reserve(wall_raw_points.size());

    const int window = 3;
    for (size_t i = 0; i < wall_raw_points.size(); i++) {
        float sum_x = 0, sum_y = 0;
        int count = 0;

        for (int j = -window; j <= window; j++) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(wall_raw_points.size())) {
                sum_x += wall_raw_points[idx].x();
                sum_y += wall_raw_points[idx].y();
                count++;
            }
        }
        smoothed.emplace_back(sum_x / static_cast<float>(count), sum_y / static_cast<float>(count));
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
        if (walls.empty() || wall_raw_points.size() == 1) {
            walls.emplace_back(simplified);
        } else {
            walls.back().points = simplified;
        }
    }
}

} // namespace

bool WorldEditorPanel::render(world::Preset& current_preset, std::vector<Eigen::Vector3f>& gt_pose_raw, core::Trajectory2D& gt_trajectory,
                              std::vector<Eigen::Vector2f>& landmarks, std::vector<core::Wall>& walls, std::vector<Eigen::Vector2f>& wall_raw_points,
                              const sensors::Camera2D& camera) {
    bool world_changed = false;

    // World preset selector
    int current_preset_idx = static_cast<int>(current_preset);
    ImGui::SetNextItemWidth(200);
    auto preset_getter = [](void*, int idx) { return world::preset_name(static_cast<world::Preset>(idx)); };
    if (ImGui::Combo("World Preset", &current_preset_idx, preset_getter, nullptr, static_cast<int>(world::Preset::COUNT))) {
        // Load preset - caller should handle this
        current_preset = static_cast<world::Preset>(current_preset_idx);
        world_changed = true;
    }
    ImGui::Separator();

    ImGui::TextWrapped("Ctrl+Click: Add new landmark | Ctrl+Drag: Draw trajectory");
    ImGui::SameLine();
    if (ImGui::Button("Clear trajectory")) {
        gt_pose_raw.clear();
        gt_trajectory = core::Trajectory2D();
        current_preset = world::Preset::Custom;
        world_changed = true;
    }

    ImGui::TextWrapped("Right-click landmark: Delete");
    ImGui::SameLine();
    if (ImGui::Button("Clear all landmarks")) {
        landmarks.clear();
        current_preset = world::Preset::Custom;
        world_changed = true;
    }

    ImGui::TextWrapped("Shift+Drag: Draw wall");
    ImGui::SameLine();
    if (ImGui::Button("Clear all walls")) {
        walls.clear();
        wall_raw_points.clear();
        current_preset = world::Preset::Custom;
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
                this->_trajectory_drag_started = true;
                this->_trajectory_drag_start_pos = ImGui::GetMousePos();
                this->_landmark_click_started = true;
                this->_landmark_click_start_pos = ImGui::GetMousePos();
            }

            if (this->_trajectory_drag_started && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                ImVec2 current_pos = ImGui::GetMousePos();
                float drag_dist = current_pos.distance(this->_trajectory_drag_start_pos);

                // If dragged more than threshold, switch to trajectory drawing mode
                if (drag_dist > 5.0f) {
                    // Clear trajectory on first drag movement
                    if (this->_landmark_click_started) {
                        gt_pose_raw.clear();
                        gt_trajectory = core::Trajectory2D();
                        this->_landmark_click_started = false;
                        current_preset = world::Preset::Custom;
                    }

                    bool should_add = true;
                    if (!gt_pose_raw.empty()) {
                        ImPlotPoint last_plot(gt_pose_raw.back().x(), gt_pose_raw.back().y());
                        ImVec2 last_px = ImPlot::PlotToPixels(last_plot);
                        ImVec2 mouse_px = ImPlot::PlotToPixels(mouse);
                        float dist = mouse_px.distance(last_px);
                        should_add = dist > 1.0f;
                    }
                    if (should_add) {
                        // Compute orientation from previous pose
                        float orientation = 0.0f;
                        if (!gt_pose_raw.empty()) {
                            Eigen::Vector2f prev(gt_pose_raw.back().x(), gt_pose_raw.back().y());
                            Eigen::Vector2f curr(mouse.x, mouse.y);
                            Eigen::Vector2f dir = curr - prev;
                            orientation = std::atan2(dir.y(), dir.x());
                        }
                        gt_pose_raw.emplace_back(static_cast<float>(mouse.x), static_cast<float>(mouse.y), orientation);
                        // Rebuild trajectory while drawing for live preview
                        build_trajectory_from_raw_poses(gt_pose_raw, gt_trajectory);
                    }
                }
            }

            // On release, if we didn't drag much, add a landmark
            if (this->_landmark_click_started && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                ImVec2 current_pos = ImGui::GetMousePos();
                float dist = current_pos.distance(this->_landmark_click_start_pos);
                if (dist < 5.0f) {
                    landmarks.emplace_back(static_cast<float>(mouse.x), static_cast<float>(mouse.y));
                    current_preset = world::Preset::Custom;
                    world_changed = true;
                }
            }
        }

        // Reset tracking on mouse release and mark world as changed if we drew something
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            if (this->_trajectory_drag_started && !this->_landmark_click_started && !gt_pose_raw.empty()) {
                world_changed = true; // Trajectory was drawn
            }
            this->_trajectory_drag_started = false;
            this->_landmark_click_started = false;
            if (this->_wall_drag_started) {
                this->_wall_drag_started = false;
                wall_raw_points.clear(); // Clear raw points after wall is finalized
                world_changed = true;    // Wall was drawn
            }
        }

        // Handle Shift+drag to draw walls
        if (is_hovered && ImGui::GetIO().KeyShift && !ImGui::GetIO().KeyCtrl) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                this->_wall_drag_started = true;
                this->_wall_drag_start_pos = ImGui::GetMousePos();
                wall_raw_points.clear();
            }

            if (this->_wall_drag_started && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                ImVec2 current_pos = ImGui::GetMousePos();
                float drag_dist = current_pos.distance(this->_wall_drag_start_pos);

                if (drag_dist > 3.0f) {
                    // Start a new wall if this is the first point
                    if (wall_raw_points.empty()) {
                        walls.emplace_back();
                        current_preset = world::Preset::Custom;
                    }

                    bool should_add = true;
                    if (!wall_raw_points.empty()) {
                        ImPlotPoint last_plot(wall_raw_points.back().x(), wall_raw_points.back().y());
                        ImVec2 last_px = ImPlot::PlotToPixels(last_plot);
                        ImVec2 mouse_px_check = ImPlot::PlotToPixels(mouse);
                        float dist = mouse_px_check.distance(last_px);
                        should_add = dist > 1.0f;
                    }
                    if (should_add) {
                        wall_raw_points.emplace_back(static_cast<float>(mouse.x), static_cast<float>(mouse.y));
                        build_wall_from_raw_points(walls, wall_raw_points);
                    }
                }
            }
        }

        // Find closest point (landmark or GT) for tooltip
        ImVec2 mouse_px = ImGui::GetMousePos();
        float closest_dist = 5.0f;
        int closest_landmark = -1;
        int closest_gt = -1;

        for (size_t i = 0; i < landmarks.size(); i++) {
            ImVec2 lm_px = ImPlot::PlotToPixels(ImPlotPoint(landmarks[i].x(), landmarks[i].y()));
            float dist = mouse_px.distance(lm_px);
            if (dist < closest_dist) {
                closest_dist = dist;
                closest_landmark = static_cast<int>(i);
                closest_gt = -1;
            }
        }

        // Check proximity to trajectory poses
        if (gt_trajectory.is_valid()) {
            for (size_t i = 0; i < gt_trajectory.num_poses(); i++) {
                Eigen::Vector2f pos = gt_trajectory.position(static_cast<float>(i));
                ImVec2 gt_px = ImPlot::PlotToPixels(ImPlotPoint(pos.x(), pos.y()));
                float dist = mouse_px.distance(gt_px);
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_gt = static_cast<int>(i);
                    closest_landmark = -1;
                }
            }
        }

        // Render trajectory
        if (gt_trajectory.is_valid()) {
            plot_2d_trajectory("Trajectory", gt_trajectory, Color::Green());

            // Extract poses from trajectory for hover detection
            std::vector<Eigen::Vector3f> trajectory_poses;
            trajectory_poses.reserve(static_cast<size_t>(gt_trajectory.max_t()) + 1);
            for (float t = 0.0f; t <= gt_trajectory.max_t(); t += 1.0f) {
                trajectory_poses.push_back(gt_trajectory.pose_vector(t));
            }
            _ui_state->handle_hovered_pose(trajectory_poses);
        }

        // While drawing, show camera at last pose; otherwise show tooltip for closest point
        bool is_drawing = this->_trajectory_drag_started && !this->_landmark_click_started;
        if (is_drawing && gt_trajectory.is_valid()) {
            // Show camera at last pose while drawing
            float last_t = gt_trajectory.max_t();
            Eigen::Vector3f pose = gt_trajectory.pose_vector(last_t);
            Eigen::Vector2f pos(pose.x(), pose.y());
            // Filter landmarks by wall occlusion, keeping track of original indices
            std::vector<sensors::CameraMeasurement> observations;
            for (size_t i = 0; i < landmarks.size(); i++) {
                if (!simulation::is_landmark_occluded(pos, landmarks[i], walls)) {
                    auto u = camera.project(pose, landmarks[i]);
                    if (u.has_value()) {
                        observations.emplace_back(u.value(), i);
                    }
                }
            }
            plot_2d_camera_frustum("##DrawingCamera", pos, pose.z(), camera.fov(), 1.0f, Color::Blue());
            plot_2d_camera_rays("##DrawingRays", pos, landmarks, observations, 1.0f);
            plot_2d_camera_observations("##DrawingObs", pos, pose.z(), camera, observations);
        } else {
            // Show tooltip for closest point and camera preview for poses
            if (closest_landmark >= 0) {
                // Count observations for this landmark (considering wall occlusion)
                int obs_count = 0;
                if (gt_trajectory.is_valid()) {
                    int num_poses = static_cast<int>(gt_trajectory.max_t()) + 1;
                    for (int t = 0; t < num_poses; t++) {
                        Eigen::Vector3f pose = gt_trajectory.pose_vector(static_cast<float>(t));
                        Eigen::Vector2f pos(pose.x(), pose.y());
                        // Check wall occlusion before projecting
                        if (simulation::is_landmark_occluded(pos, landmarks[closest_landmark], walls))
                            continue;
                        auto u = camera.project(pose, landmarks[closest_landmark]);
                        if (u.has_value()) {
                            obs_count++;
                            // Only show observation for the hovered landmark
                            std::vector<sensors::CameraMeasurement> single_obs = {{u.value(), static_cast<size_t>(closest_landmark)}};
                            plot_2d_camera_frustum("##LandmarkHoverCamera", pos, pose.z(), camera.fov(), 1.0f, Color::Blue());
                            plot_2d_camera_rays("##LandmarkHoverRays", pos, landmarks, single_obs, 1.0f);
                            plot_2d_camera_observations("##LandmarkHoverObs", pos, pose.z(), camera, single_obs);
                        }
                    }
                }
                ImGui::SetTooltip("Landmark %d\nPos: (%.2f, %.2f)\nObservations: %d", closest_landmark, landmarks[closest_landmark].x(),
                                  landmarks[closest_landmark].y(), obs_count);
            } else if (closest_gt >= 0 && gt_trajectory.is_valid()) {
                Eigen::Vector3f pose = gt_trajectory.pose_vector(static_cast<float>(closest_gt));
                // Draw camera frustum with projected landmarks (filtered by walls)
                Eigen::Vector2f pos(pose.x(), pose.y());
                std::vector<sensors::CameraMeasurement> observations;
                for (size_t i = 0; i < landmarks.size(); i++) {
                    if (!simulation::is_landmark_occluded(pos, landmarks[i], walls)) {
                        auto u = camera.project(pose, landmarks[i]);
                        if (u.has_value()) {
                            observations.emplace_back(u.value(), i);
                        }
                    }
                }
                ImGui::SetTooltip("GT Pose %d\nPos: (%.2f, %.2f)\nOrientation: %.2f°\nObservations: %zu", closest_gt, pose.x(), pose.y(),
                                  pose.z() * core::RAD_TO_DEG, observations.size());
                plot_2d_camera_frustum("##HoverCamera", pos, pose.z(), camera.fov(), 1.0f, Color::Blue());
                plot_2d_camera_rays("##HoverRays", pos, landmarks, observations, 1.0f);
                plot_2d_camera_observations("##HoverObs", pos, pose.z(), camera, observations);
            }
        }

        // Render landmarks as draggable points with context menu
        int landmark_to_delete = -1;
        for (size_t i = 0; i < landmarks.size(); i++) {
            double x = landmarks[i].x();
            double y = landmarks[i].y();
            Color lm_color = Color::Random(i);
            if (ImPlot::DragPoint(static_cast<int>(i), &x, &y, ImVec4(lm_color), 4.0f)) {
                landmarks[i] = Eigen::Vector2f(x, y);
                current_preset = world::Preset::Custom;
                world_changed = true;
            }

            // Check if mouse is hovering this landmark for context menu
            ImVec2 lm_px = ImPlot::PlotToPixels(ImPlotPoint(x, y));
            float dist = mouse_px.distance(lm_px);
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
            landmarks.erase(landmarks.begin() + landmark_to_delete);
            current_preset = world::Preset::Custom;
            world_changed = true;
        }

        // Render walls (plain white)
        int wall_to_delete = -1;
        for (size_t w = 0; w < walls.size(); w++) {
            const auto& wall = walls[w];
            if (wall.points.size() >= 2) {
                plot_2d_line("##Wall" + std::to_string(w), wall.points, Color::White(), 2.0f);

                // Check if mouse is near any segment of this wall for context menu
                for (size_t i = 0; i + 1 < wall.points.size(); i++) {
                    ImVec2 p1_px = ImPlot::PlotToPixels(ImPlotPoint(wall.points[i].x(), wall.points[i].y()));
                    ImVec2 p2_px = ImPlot::PlotToPixels(ImPlotPoint(wall.points[i + 1].x(), wall.points[i + 1].y()));

                    // Distance from mouse to line segment
                    ImVec2 seg = p2_px - p1_px;
                    float len_sq = seg.length_squared();
                    float t = 0.0f;
                    if (len_sq > 0) {
                        t = std::clamp((mouse_px - p1_px).dot(seg) / len_sq, 0.0f, 1.0f);
                    }
                    ImVec2 closest = p1_px + seg * t;
                    float dist = mouse_px.distance(closest);

                    if (dist < 10.0f && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                        ImGui::OpenPopup(("WallContext" + std::to_string(w)).c_str());
                        break;
                    }
                }
            }

            if (ImGui::BeginPopup(("WallContext" + std::to_string(w)).c_str())) {
                ImGui::Text("Wall %zu (%zu segments)", w, !wall.points.empty() ? wall.points.size() - 1 : 0);
                ImGui::Separator();
                if (ImGui::MenuItem("Delete")) {
                    wall_to_delete = static_cast<int>(w);
                }
                ImGui::EndPopup();
            }
        }
        if (wall_to_delete >= 0) {
            walls.erase(walls.begin() + wall_to_delete);
            current_preset = world::Preset::Custom;
            world_changed = true;
        }

        ImPlot::EndPlot();
    }
    return world_changed;
}

} // namespace gui
