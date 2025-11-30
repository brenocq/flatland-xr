// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <gtest/gtest.h>
#include <world/world.hpp>

using namespace world;

// ============================================================================
// Preset enum tests
// ============================================================================

TEST(WorldPreset, PresetNames) {
    EXPECT_STREQ(preset_name(Preset::Custom), "Custom");
    EXPECT_STREQ(preset_name(Preset::ASquaresHouse), "A Square's House");
    EXPECT_STREQ(preset_name(Preset::VisitFromSphere), "Visit from Sphere");
    EXPECT_STREQ(preset_name(Preset::HallOfCouncil), "Hall of Council");
}

TEST(WorldPreset, PresetCount) { EXPECT_EQ(static_cast<int>(Preset::COUNT), 4); }

// ============================================================================
// World struct tests
// ============================================================================

TEST(World, DefaultIsEmpty) {
    World world;
    EXPECT_TRUE(world.empty());
    EXPECT_TRUE(world.landmarks.empty());
    EXPECT_TRUE(world.walls.empty());
    EXPECT_TRUE(world.trajectory_keypoints.empty());
}

TEST(World, ClearRemovesAllData) {
    World world;
    world.landmarks.push_back(Eigen::Vector2f(1, 2));
    world.walls.push_back(core::Wall({{0, 0}, {1, 1}}));
    world.trajectory_keypoints.push_back(Eigen::Vector3f(0, 0, 0));

    world.clear();

    EXPECT_TRUE(world.empty());
}

TEST(World, EmptyChecksAllFields) {
    World world;

    world.landmarks.push_back(Eigen::Vector2f(1, 2));
    EXPECT_FALSE(world.empty());
    world.landmarks.clear();

    world.walls.push_back(core::Wall());
    EXPECT_FALSE(world.empty());
    world.walls.clear();

    world.trajectory_keypoints.push_back(Eigen::Vector3f(0, 0, 0));
    EXPECT_FALSE(world.empty());
}

// ============================================================================
// load_preset tests
// ============================================================================

TEST(LoadPreset, CustomReturnsEmpty) {
    World world = load_preset(Preset::Custom);
    EXPECT_TRUE(world.empty());
}

TEST(LoadPreset, ASquaresHouseHasData) {
    World world = load_preset(Preset::ASquaresHouse);

    EXPECT_FALSE(world.empty());
    EXPECT_FALSE(world.landmarks.empty());
    EXPECT_FALSE(world.walls.empty());
    EXPECT_FALSE(world.trajectory_keypoints.empty());
}

TEST(LoadPreset, VisitFromSphereHasData) {
    World world = load_preset(Preset::VisitFromSphere);

    EXPECT_FALSE(world.empty());
    EXPECT_FALSE(world.landmarks.empty());
    EXPECT_FALSE(world.walls.empty());
    EXPECT_FALSE(world.trajectory_keypoints.empty());
}

TEST(LoadPreset, HallOfCouncilHasData) {
    World world = load_preset(Preset::HallOfCouncil);

    EXPECT_FALSE(world.empty());
    EXPECT_FALSE(world.landmarks.empty());
    EXPECT_FALSE(world.walls.empty());
    EXPECT_FALSE(world.trajectory_keypoints.empty());
}

TEST(LoadPreset, LandmarksAreFinite) {
    for (int i = 1; i < static_cast<int>(Preset::COUNT); i++) {
        World world = load_preset(static_cast<Preset>(i));
        for (const auto& lm : world.landmarks) {
            EXPECT_TRUE(std::isfinite(lm.x()));
            EXPECT_TRUE(std::isfinite(lm.y()));
        }
    }
}

TEST(LoadPreset, WallPointsAreFinite) {
    for (int i = 1; i < static_cast<int>(Preset::COUNT); i++) {
        World world = load_preset(static_cast<Preset>(i));
        for (const auto& wall : world.walls) {
            for (const auto& pt : wall.points) {
                EXPECT_TRUE(std::isfinite(pt.x()));
                EXPECT_TRUE(std::isfinite(pt.y()));
            }
        }
    }
}

TEST(LoadPreset, TrajectoryKeypointsAreFinite) {
    for (int i = 1; i < static_cast<int>(Preset::COUNT); i++) {
        World world = load_preset(static_cast<Preset>(i));
        for (const auto& kp : world.trajectory_keypoints) {
            EXPECT_TRUE(std::isfinite(kp.x()));
            EXPECT_TRUE(std::isfinite(kp.y()));
            EXPECT_TRUE(std::isfinite(kp.z()));
        }
    }
}

// ============================================================================
// interpolate_trajectory tests
// ============================================================================

TEST(InterpolateTrajectory, EmptyKeypointsReturnsEmpty) {
    std::vector<Eigen::Vector3f> empty;
    auto result = interpolate_trajectory(empty);
    EXPECT_TRUE(result.empty());
}

TEST(InterpolateTrajectory, SingleKeypointReturnsEmpty) {
    std::vector<Eigen::Vector3f> single = {{0, 0, 0}};
    auto result = interpolate_trajectory(single);
    EXPECT_TRUE(result.empty());
}

TEST(InterpolateTrajectory, TwoKeypointsGeneratesPoses) {
    std::vector<Eigen::Vector3f> keypoints = {
        {0, 0, 0},
        {10, 0, 0},
    };
    auto result = interpolate_trajectory(keypoints, 10);

    EXPECT_GE(result.size(), 2u);
}

TEST(InterpolateTrajectory, RequestedNumberOfPoses) {
    std::vector<Eigen::Vector3f> keypoints = {
        {0, 0, 0},
        {5, 0, 0},
        {10, 0, 0},
    };
    auto result = interpolate_trajectory(keypoints, 60);

    // Should be close to requested number (may vary slightly due to interpolation)
    EXPECT_GE(result.size(), 50u);
    EXPECT_LE(result.size(), 70u);
}

TEST(InterpolateTrajectory, FirstPoseMatchesFirstKeypoint) {
    std::vector<Eigen::Vector3f> keypoints = {
        {1, 2, 0.5f},
        {5, 3, 1.0f},
    };
    auto result = interpolate_trajectory(keypoints, 10);

    ASSERT_FALSE(result.empty());
    EXPECT_NEAR(result.front().x(), 1.0f, 0.1f);
    EXPECT_NEAR(result.front().y(), 2.0f, 0.1f);
}

TEST(InterpolateTrajectory, LastPoseMatchesLastKeypoint) {
    std::vector<Eigen::Vector3f> keypoints = {
        {0, 0, 0},
        {10, 5, 1.0f},
    };
    auto result = interpolate_trajectory(keypoints, 10);

    ASSERT_FALSE(result.empty());
    EXPECT_NEAR(result.back().x(), 10.0f, 0.5f);
    EXPECT_NEAR(result.back().y(), 5.0f, 0.5f);
}

TEST(InterpolateTrajectory, PosesAreMonotonic) {
    std::vector<Eigen::Vector3f> keypoints = {
        {0, 0, 0},
        {5, 0, 0},
        {10, 0, 0},
    };
    auto result = interpolate_trajectory(keypoints, 20);

    // X should be monotonically increasing for this straight path
    for (size_t i = 1; i < result.size(); i++) {
        EXPECT_GE(result[i].x(), result[i - 1].x() - 0.1f);
    }
}

TEST(InterpolateTrajectory, OrientationPointsAlongPath) {
    std::vector<Eigen::Vector3f> keypoints = {
        {0, 0, 0},
        {10, 0, 0}, // Moving right, orientation should be ~0
    };
    auto result = interpolate_trajectory(keypoints, 10);

    for (const auto& pose : result) {
        // Orientation should be close to 0 (pointing right)
        EXPECT_NEAR(pose.z(), 0.0f, 0.5f);
    }
}

TEST(InterpolateTrajectory, CurvedPathOrientationChanges) {
    std::vector<Eigen::Vector3f> keypoints = {
        {0, 0, 0},
        {5, 0, 0},
        {5, 5, M_PI / 2}, // Turn 90 degrees
    };
    auto result = interpolate_trajectory(keypoints, 20);

    // First pose should have orientation ~0, last should have ~pi/2
    ASSERT_FALSE(result.empty());
    // Just verify we have poses with varying orientations
    bool has_varying_orientation = false;
    for (size_t i = 1; i < result.size(); i++) {
        if (std::abs(result[i].z() - result[0].z()) > 0.1f) {
            has_varying_orientation = true;
            break;
        }
    }
    EXPECT_TRUE(has_varying_orientation);
}
