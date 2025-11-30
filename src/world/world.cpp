// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include "world.hpp"
#include <cmath>

namespace world {

const char* preset_name(Preset preset) {
    switch (preset) {
        case Preset::Custom: return "Custom";
        case Preset::ASquaresHouse: return "A Square's House";
        case Preset::VisitFromSphere: return "Visit from Sphere";
        case Preset::HallOfCouncil: return "Hall of Council";
        default: return "Unknown";
    }
}

namespace {

/// Helper to create a regular polygon (habitant) as walls with landmarks at vertices
void create_polygon(World& world, Eigen::Vector2f center, float size, int sides, float rotation = 0.0f, float landmark_offset = 0.15f) {
    std::vector<Eigen::Vector2f> vertices;
    for (int i = 0; i < sides; i++) {
        float angle = rotation + i * 2.0f * M_PI / sides;
        vertices.push_back(center + Eigen::Vector2f(size * std::cos(angle), size * std::sin(angle)));
    }
    // Create walls
    for (int i = 0; i < sides; i++) {
        core::Wall wall;
        wall.points.push_back(vertices[i]);
        wall.points.push_back(vertices[(i + 1) % sides]);
        world.walls.push_back(wall);
    }
    // Create landmarks slightly outside each vertex
    for (int i = 0; i < sides; i++) {
        Eigen::Vector2f dir = (vertices[i] - center).normalized();
        world.landmarks.push_back(vertices[i] + dir * landmark_offset);
    }
}

/// Helper to create a line (wife/daughter in Flatland) as a wall with landmarks at ends
void create_line(World& world, Eigen::Vector2f center, float length, float angle, float landmark_offset = 0.15f) {
    Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
    Eigen::Vector2f p1 = center - dir * (length / 2.0f);
    Eigen::Vector2f p2 = center + dir * (length / 2.0f);
    core::Wall wall;
    wall.points.push_back(p1);
    wall.points.push_back(p2);
    world.walls.push_back(wall);
    // Landmarks at the endpoints, slightly offset outward
    world.landmarks.push_back(p1 - dir * landmark_offset);
    world.landmarks.push_back(p2 + dir * landmark_offset);
}

/// Helper to create a circle (priest/ruler) approximated as many-sided polygon
void create_circle(World& world, Eigen::Vector2f center, float radius, int segments = 16, float landmark_offset = 0.15f) {
    std::vector<Eigen::Vector2f> vertices;
    for (int i = 0; i < segments; i++) {
        float angle = i * 2.0f * M_PI / segments;
        vertices.push_back(center + Eigen::Vector2f(radius * std::cos(angle), radius * std::sin(angle)));
    }
    // Create walls
    for (int i = 0; i < segments; i++) {
        core::Wall wall;
        wall.points.push_back(vertices[i]);
        wall.points.push_back(vertices[(i + 1) % segments]);
        world.walls.push_back(wall);
    }
    // Add landmarks at cardinal points only (to avoid too many)
    for (int i = 0; i < 4; i++) {
        float angle = i * M_PI / 2.0f;
        Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
        world.landmarks.push_back(center + dir * (radius + landmark_offset));
    }
}

World load_a_squares_house() {
    World world;

    // A Square's pentagonal house - higher class Flatlanders have more sides
    const float house_radius = 10.0f;
    std::vector<Eigen::Vector2f> pentagon;
    for (int i = 0; i < 5; i++) {
        float angle = M_PI / 2.0f + i * 2.0f * M_PI / 5.0f; // Start from top
        pentagon.push_back(Eigen::Vector2f(house_radius * std::cos(angle), house_radius * std::sin(angle)));
    }
    for (int i = 0; i < 5; i++) {
        core::Wall wall;
        wall.points.push_back(pentagon[i]);
        wall.points.push_back(pentagon[(i + 1) % 5]);
        world.walls.push_back(wall);
    }

    // A Square (narrator) - a square shape in the center-left area
    create_polygon(world, Eigen::Vector2f(-3.0f, 1.0f), 0.8f, 4, M_PI / 4.0f);

    // Wife (line) near southern door - lines are dangerous in Flatland!
    create_line(world, Eigen::Vector2f(0.0f, -6.0f), 1.5f, M_PI / 2.0f);

    // Son 1 - an equilateral triangle (lower class)
    create_polygon(world, Eigen::Vector2f(-4.0f, 4.0f), 0.7f, 3, -M_PI / 2.0f);

    // Son 2 - a square (same class as father)
    create_polygon(world, Eigen::Vector2f(4.0f, 3.0f), 0.6f, 4, M_PI / 4.0f);

    // Grandson - isosceles triangle (soldier class)
    create_polygon(world, Eigen::Vector2f(2.0f, -3.0f), 0.5f, 3, M_PI / 2.0f);

    // Furniture corners as small triangles
    create_polygon(world, Eigen::Vector2f(-5.0f, -2.0f), 0.4f, 3, 0.0f);
    create_polygon(world, Eigen::Vector2f(5.0f, -2.0f), 0.4f, 3, M_PI);

    // Trajectory keypoints: A Square moving around his house visiting family
    world.trajectory_keypoints = {
        {-6.0f, -4.0f, M_PI / 4.0f}, {-5.0f, -1.0f, M_PI / 2.0f},        {-5.0f, 3.0f, M_PI / 3.0f},  {-3.0f, 5.0f, 0.0f},
        {2.0f, 5.0f, -M_PI / 4.0f},  {5.0f, 2.0f, -M_PI / 2.0f},         {5.0f, -2.0f, -M_PI / 2.0f}, {3.0f, -5.0f, M_PI},
        {-1.0f, -5.0f, M_PI},        {-4.0f, -3.0f, 2.0f * M_PI / 3.0f},
    };

    return world;
}

World load_visit_from_sphere() {
    World world;

    // The Sphere visits A Square - the climactic scene
    // Simple rectangular room
    float w = 12.0f, h = 10.0f;
    world.walls.push_back(core::Wall({Eigen::Vector2f(-w, -h), Eigen::Vector2f(w, -h)})); // Bottom
    world.walls.push_back(core::Wall({Eigen::Vector2f(w, -h), Eigen::Vector2f(w, h)}));   // Right
    world.walls.push_back(core::Wall({Eigen::Vector2f(w, h), Eigen::Vector2f(-w, h)}));   // Top
    world.walls.push_back(core::Wall({Eigen::Vector2f(-w, h), Eigen::Vector2f(-w, -h)})); // Left

    // The Sphere - appears as a circle that grows and shrinks
    create_circle(world, Eigen::Vector2f(5.0f, 0.0f), 2.5f, 20);

    // A Square (the narrator) observing in amazement
    create_polygon(world, Eigen::Vector2f(-6.0f, 0.0f), 0.8f, 4, M_PI / 4.0f);

    // Wife hiding in corner (lines are shy/dangerous)
    create_line(world, Eigen::Vector2f(-9.0f, -7.0f), 1.2f, M_PI / 4.0f);

    // Furniture - small triangular tables
    create_polygon(world, Eigen::Vector2f(-8.0f, 6.0f), 0.5f, 3, 0.0f);
    create_polygon(world, Eigen::Vector2f(8.0f, 6.0f), 0.5f, 3, M_PI);
    create_polygon(world, Eigen::Vector2f(8.0f, -6.0f), 0.5f, 3, M_PI);

    // Trajectory keypoints: A Square approaching and circling the Sphere
    world.trajectory_keypoints = {
        {-10.0f, 0.0f, 0.0f},        {-7.0f, 0.0f, 0.0f},
        {-4.0f, 2.0f, M_PI / 6.0f},  {-1.0f, 5.0f, M_PI / 4.0f},
        {3.0f, 6.0f, 0.0f},          {7.0f, 4.0f, -M_PI / 4.0f},
        {9.0f, 0.0f, -M_PI / 2.0f},  {7.0f, -4.0f, -3.0f * M_PI / 4.0f},
        {3.0f, -6.0f, M_PI},         {-1.0f, -5.0f, 3.0f * M_PI / 4.0f},
        {-4.0f, -2.0f, M_PI / 2.0f}, {-6.0f, 1.0f, M_PI / 4.0f},
    };

    return world;
}

World load_hall_of_council() {
    World world;

    // The grand circular hall where the ruling Circles meet
    const float outer_radius = 18.0f;
    const int sides = 12;
    std::vector<Eigen::Vector2f> hall;
    for (int i = 0; i < sides; i++) {
        float angle = i * 2.0f * M_PI / sides;
        hall.push_back(Eigen::Vector2f(outer_radius * std::cos(angle), outer_radius * std::sin(angle)));
    }
    for (int i = 0; i < sides; i++) {
        core::Wall wall;
        wall.points.push_back(hall[i]);
        wall.points.push_back(hall[(i + 1) % sides]);
        world.walls.push_back(wall);
    }

    // Inner podium (hexagon)
    const float inner_radius = 5.0f;
    std::vector<Eigen::Vector2f> podium;
    for (int i = 0; i < 6; i++) {
        float angle = M_PI / 6.0f + i * 2.0f * M_PI / 6.0f;
        podium.push_back(Eigen::Vector2f(inner_radius * std::cos(angle), inner_radius * std::sin(angle)));
    }
    for (int i = 0; i < 6; i++) {
        core::Wall wall;
        wall.points.push_back(podium[i]);
        wall.points.push_back(podium[(i + 1) % 6]);
        world.walls.push_back(wall);
    }

    // The ruling Circles (high priests)
    create_circle(world, Eigen::Vector2f(11.0f, 0.0f), 1.2f, 12);
    create_circle(world, Eigen::Vector2f(-11.0f, 0.0f), 1.2f, 12);
    create_circle(world, Eigen::Vector2f(0.0f, 11.0f), 1.2f, 12);
    create_circle(world, Eigen::Vector2f(0.0f, -11.0f), 1.2f, 12);
    create_circle(world, Eigen::Vector2f(8.0f, 8.0f), 1.0f, 12);
    create_circle(world, Eigen::Vector2f(-8.0f, 8.0f), 1.0f, 12);
    create_circle(world, Eigen::Vector2f(-8.0f, -8.0f), 1.0f, 12);
    create_circle(world, Eigen::Vector2f(8.0f, -8.0f), 1.0f, 12);

    // Isosceles guards (triangles) at entrances
    create_polygon(world, Eigen::Vector2f(15.0f, 0.0f), 0.8f, 3, M_PI);
    create_polygon(world, Eigen::Vector2f(-15.0f, 0.0f), 0.8f, 3, 0.0f);

    // Central speaker - a high-ranking polygon (octagon)
    create_polygon(world, Eigen::Vector2f(0.0f, 0.0f), 1.0f, 8, M_PI / 8.0f);

    // A Square observing from the audience
    create_polygon(world, Eigen::Vector2f(13.0f, 5.0f), 0.6f, 4, M_PI / 4.0f);

    // Trajectory keypoints: A Square entering and observing the council
    world.trajectory_keypoints = {
        {16.0f, 2.0f, M_PI},
        {14.0f, 3.0f, 2.0f * M_PI / 3.0f},
        {10.0f, 6.0f, 2.0f * M_PI / 3.0f},
        {6.0f, 8.0f, M_PI / 2.0f},
        {0.0f, 9.0f, M_PI / 2.0f},
        {-6.0f, 8.0f, 2.0f * M_PI / 3.0f},
        {-10.0f, 5.0f, 5.0f * M_PI / 6.0f},
        {-12.0f, 0.0f, M_PI},
        {-10.0f, -5.0f, -5.0f * M_PI / 6.0f},
        {-6.0f, -8.0f, -2.0f * M_PI / 3.0f},
        {0.0f, -9.0f, -M_PI / 2.0f},
        {6.0f, -8.0f, -M_PI / 3.0f},
        {10.0f, -5.0f, -M_PI / 6.0f},
        {13.0f, 0.0f, 0.0f},
    };

    return world;
}

} // anonymous namespace

World load_preset(Preset preset) {
    switch (preset) {
        case Preset::ASquaresHouse: return load_a_squares_house();
        case Preset::VisitFromSphere: return load_visit_from_sphere();
        case Preset::HallOfCouncil: return load_hall_of_council();
        default: return World{};
    }
}

std::vector<Eigen::Vector3f> interpolate_trajectory(const std::vector<Eigen::Vector3f>& keypoints, int num_poses) {
    if (keypoints.size() < 2) {
        return keypoints;
    }

    std::vector<Eigen::Vector3f> dense_poses;
    dense_poses.reserve(num_poses);

    // Calculate total path length for uniform distribution
    float total_length = 0.0f;
    std::vector<float> segment_lengths;
    for (size_t i = 0; i + 1 < keypoints.size(); i++) {
        float dx = keypoints[i + 1].x() - keypoints[i].x();
        float dy = keypoints[i + 1].y() - keypoints[i].y();
        float len = std::sqrt(dx * dx + dy * dy);
        segment_lengths.push_back(len);
        total_length += len;
    }

    // Generate poses uniformly along the path
    float step = total_length / (num_poses - 1);
    float accumulated = 0.0f;
    size_t seg_idx = 0;

    for (int i = 0; i < num_poses; i++) {
        float target_dist = i * step;

        // Find which segment we're on
        while (seg_idx < segment_lengths.size() - 1 && accumulated + segment_lengths[seg_idx] < target_dist) {
            accumulated += segment_lengths[seg_idx];
            seg_idx++;
        }

        // Interpolate within segment
        float seg_len = segment_lengths[seg_idx];
        float t = (seg_len > 0.0f) ? (target_dist - accumulated) / seg_len : 0.0f;
        t = std::clamp(t, 0.0f, 1.0f);

        float x = keypoints[seg_idx].x() + t * (keypoints[seg_idx + 1].x() - keypoints[seg_idx].x());
        float y = keypoints[seg_idx].y() + t * (keypoints[seg_idx + 1].y() - keypoints[seg_idx].y());

        // Compute orientation from movement direction
        float orientation = 0.0f;
        if (!dense_poses.empty()) {
            float dx = x - dense_poses.back().x();
            float dy = y - dense_poses.back().y();
            if (dx != 0 || dy != 0) {
                orientation = std::atan2(dy, dx);
            } else {
                orientation = dense_poses.back().z();
            }
        } else if (keypoints.size() > 1) {
            float dx = keypoints[1].x() - keypoints[0].x();
            float dy = keypoints[1].y() - keypoints[0].y();
            orientation = std::atan2(dy, dx);
        }

        dense_poses.push_back(Eigen::Vector3f(x, y, orientation));
    }

    return dense_poses;
}

} // namespace world
