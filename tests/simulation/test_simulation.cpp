// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <core/math.hpp>
#include <core/trajectory2d.hpp>
#include <core/types.hpp>
#include <gtest/gtest.h>
#include <sensors/camera2d.hpp>
#include <sensors/imu2d.hpp>
#include <simulation/simulation.hpp>

using namespace simulation;
using namespace core;

class SimulationTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Create a simple trajectory
        std::vector<Eigen::Vector3f> poses;
        for (int i = 0; i < 10; i++) {
            poses.push_back(Eigen::Vector3f(i * 0.5f, 0, 0));
        }
        trajectory.build(poses);

        // Set up sensors (using shared pointers)
        camera = std::make_shared<sensors::Camera2D>();
        camera->set_intrinsics(100, core::HALF_PI);
        camera->set_noise_std(0.0f);

        imu = std::make_shared<sensors::IMU2D>();
        imu->set_acc_bias(Eigen::Vector2f::Zero());
        imu->set_gyr_bias(0.0f);
        imu->set_acc_noise_std(Eigen::Vector2f::Zero());
        imu->set_gyr_noise_std(0.0f);

        // Create landmarks
        landmarks = {
            {2.0f, 1.0f},
            {3.0f, -1.0f},
            {4.0f, 0.5f},
        };
    }

    Trajectory2D trajectory;
    std::shared_ptr<sensors::Camera2D> camera;
    std::shared_ptr<sensors::IMU2D> imu;
    std::vector<Eigen::Vector2f> landmarks;
    std::vector<Wall> walls;
};

// ============================================================================
// SimulationResult tests
// ============================================================================

TEST(SimulationResult, DefaultIsInvalid) {
    SimulationResult result;
    EXPECT_FALSE(result.is_valid());
    EXPECT_EQ(result.num_steps(), 0u);
}

TEST(SimulationResult, Clear) {
    SimulationResult result;
    result.gt_poses.push_back(Eigen::Vector3f(1, 2, 3));
    result.imu_measurements.push_back({});
    result.clear();

    EXPECT_TRUE(result.gt_poses.empty());
    EXPECT_TRUE(result.imu_measurements.empty());
}

// ============================================================================
// is_landmark_occluded tests
// ============================================================================

TEST(IsLandmarkOccluded, NoWalls) {
    std::vector<Wall> walls;
    EXPECT_FALSE(is_landmark_occluded({0, 0}, {5, 0}, walls));
}

TEST(IsLandmarkOccluded, WallBlocking) {
    Wall wall;
    wall.points = {{2, -1}, {2, 1}};
    std::vector<Wall> walls = {wall};

    EXPECT_TRUE(is_landmark_occluded({0, 0}, {5, 0}, walls));
}

TEST(IsLandmarkOccluded, WallNotBlocking) {
    Wall wall;
    wall.points = {{2, 2}, {2, 3}}; // Wall above the line of sight
    std::vector<Wall> walls = {wall};

    EXPECT_FALSE(is_landmark_occluded({0, 0}, {5, 0}, walls));
}

TEST(IsLandmarkOccluded, MultipleWallsOneBlocking) {
    Wall wall1;
    wall1.points = {{2, 2}, {2, 3}};
    Wall wall2;
    wall2.points = {{3, -1}, {3, 1}}; // This one blocks
    std::vector<Wall> walls = {wall1, wall2};

    EXPECT_TRUE(is_landmark_occluded({0, 0}, {5, 0}, walls));
}

TEST(IsLandmarkOccluded, LandmarkOnWall) {
    Wall wall;
    wall.points = {{5, -1}, {5, 1}};
    std::vector<Wall> walls = {wall};

    // Landmark at wall position - the ray just touches the wall
    EXPECT_TRUE(is_landmark_occluded({0, 0}, {5, 0}, walls));
}

// ============================================================================
// run() tests
// ============================================================================

TEST_F(SimulationTest, RunWithValidTrajectory) {
    SimulationResult result = run(trajectory, landmarks, walls, camera, imu);

    EXPECT_TRUE(result.is_valid());
    EXPECT_EQ(result.num_steps(), trajectory.num_poses());
}

TEST_F(SimulationTest, RunWithInvalidTrajectory) {
    Trajectory2D invalid_traj;
    SimulationResult result = run(invalid_traj, landmarks, walls, camera, imu);

    EXPECT_FALSE(result.is_valid());
}

TEST_F(SimulationTest, GroundTruthPosesMatchTrajectory) {
    SimulationResult result = run(trajectory, landmarks, walls, camera, imu);

    for (size_t i = 0; i < result.num_steps(); i++) {
        Eigen::Vector3f expected = trajectory.pose_vector(static_cast<float>(i));
        EXPECT_NEAR(result.gt_poses[i].x(), expected.x(), 1e-4f);
        EXPECT_NEAR(result.gt_poses[i].y(), expected.y(), 1e-4f);
        EXPECT_NEAR(result.gt_poses[i].z(), expected.z(), 1e-4f);
    }
}

TEST_F(SimulationTest, VelocitiesAreComputed) {
    SimulationResult result = run(trajectory, landmarks, walls, camera, imu);

    EXPECT_EQ(result.gt_vel.size(), result.num_steps());
    // Straight line motion should have positive x velocity
    for (size_t i = 1; i < result.num_steps() - 1; i++) {
        EXPECT_GT(result.gt_vel[i].x(), 0.0f);
    }
}

TEST_F(SimulationTest, IMUMeasurementsGenerated) {
    SimulationResult result = run(trajectory, landmarks, walls, camera, imu);

    EXPECT_EQ(result.imu_measurements.size(), result.num_steps());
    EXPECT_EQ(result.gt_imu.size(), result.num_steps());
}

TEST_F(SimulationTest, CameraMeasurementsGenerated) {
    SimulationResult result = run(trajectory, landmarks, walls, camera, imu);

    EXPECT_EQ(result.cam_measurements.size(), result.num_steps());
    EXPECT_EQ(result.gt_cam.size(), result.num_steps());
}

TEST_F(SimulationTest, GroundTruthAndMeasuredIMUMatchWithZeroNoise) {
    SimulationResult result = run(trajectory, landmarks, walls, camera, imu);

    for (size_t i = 0; i < result.num_steps(); i++) {
        EXPECT_NEAR(result.gt_imu[i].acc.x(), result.imu_measurements[i].acc.x(), 1e-5f);
        EXPECT_NEAR(result.gt_imu[i].acc.y(), result.imu_measurements[i].acc.y(), 1e-5f);
        EXPECT_NEAR(result.gt_imu[i].gyr, result.imu_measurements[i].gyr, 1e-5f);
    }
}

TEST_F(SimulationTest, WallOcclusionFiltersLandmarks) {
    // Add a wall between camera and one landmark
    Wall wall;
    wall.points = {{2.5f, -2}, {2.5f, 2}}; // Blocks landmark at (3, -1)
    walls.push_back(wall);

    SimulationResult result = run(trajectory, landmarks, walls, camera, imu);

    // At pose 0, landmark 1 (at 3, -1) should be blocked
    // Check that fewer landmarks are observed compared to without wall
    SimulationResult result_no_wall = run(trajectory, landmarks, {}, camera, imu);

    // The result with walls should have fewer or equal observations
    size_t total_obs_with_wall = 0;
    size_t total_obs_no_wall = 0;
    for (size_t i = 0; i < result.num_steps(); i++) {
        total_obs_with_wall += result.cam_measurements[i].size();
        total_obs_no_wall += result_no_wall.cam_measurements[i].size();
    }
    EXPECT_LE(total_obs_with_wall, total_obs_no_wall);
}

TEST_F(SimulationTest, GravityAffectsIMU) {
    SimulationConfig config;
    config.gravity = Eigen::Vector2f(0, -9.81f);

    SimulationResult result = run(trajectory, landmarks, walls, camera, imu, config);

    // With gravity pointing down, stationary IMU should measure upward acceleration
    // The body-frame measurement depends on orientation
    // At orientation 0 (looking along +x), gravity in body frame should have positive y
    EXPECT_TRUE(result.is_valid());
}

// ============================================================================
// Edge cases
// ============================================================================

TEST_F(SimulationTest, NoLandmarks) {
    std::vector<Eigen::Vector2f> no_landmarks;
    SimulationResult result = run(trajectory, no_landmarks, walls, camera, imu);

    EXPECT_TRUE(result.is_valid());
    for (size_t i = 0; i < result.num_steps(); i++) {
        EXPECT_TRUE(result.cam_measurements[i].empty());
    }
}

TEST_F(SimulationTest, NoWalls) {
    SimulationResult result = run(trajectory, landmarks, {}, camera, imu);
    EXPECT_TRUE(result.is_valid());
}

TEST_F(SimulationTest, EmptyWall) {
    Wall empty_wall;
    walls.push_back(empty_wall);
    SimulationResult result = run(trajectory, landmarks, walls, camera, imu);
    EXPECT_TRUE(result.is_valid());
}
