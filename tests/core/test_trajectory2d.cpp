// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <core/math.hpp>
#include <core/trajectory2d.hpp>
#include <gtest/gtest.h>

using namespace core;

class Trajectory2DTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Create a simple straight-line trajectory
        straight_poses = {
            Eigen::Vector3f(0, 0, 0),
            Eigen::Vector3f(1, 0, 0),
            Eigen::Vector3f(2, 0, 0),
            Eigen::Vector3f(3, 0, 0),
        };

        // Create a circular arc trajectory
        circular_poses.clear();
        for (int i = 0; i <= 8; i++) {
            float angle = static_cast<float>(i) * core::PI / 8.0f;
            float x = std::cos(angle);
            float y = std::sin(angle);
            circular_poses.push_back(Eigen::Vector3f(x, y, angle + core::HALF_PI));
        }
    }

    std::vector<Eigen::Vector3f> straight_poses;
    std::vector<Eigen::Vector3f> circular_poses;
};

// ============================================================================
// Basic validity tests
// ============================================================================

TEST_F(Trajectory2DTest, DefaultConstructorInvalid) {
    Trajectory2D traj;
    EXPECT_FALSE(traj.is_valid());
}

TEST_F(Trajectory2DTest, SinglePoseInvalid) {
    Trajectory2D traj;
    std::vector<Eigen::Vector3f> single = {Eigen::Vector3f(0, 0, 0)};
    traj.build(single);
    EXPECT_FALSE(traj.is_valid());
}

TEST_F(Trajectory2DTest, TwoPosesValid) {
    Trajectory2D traj;
    std::vector<Eigen::Vector3f> two = {
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(1, 0, 0),
    };
    traj.build(two);
    EXPECT_TRUE(traj.is_valid());
    EXPECT_EQ(traj.num_poses(), 2);
}

TEST_F(Trajectory2DTest, BuildFromPose2DVector) {
    Trajectory2D traj;
    std::vector<Pose2D> poses = {
        Pose2D(0, 0, 0),
        Pose2D(1, 1, 0.5f),
        Pose2D(2, 2, 1.0f),
    };
    traj.build(poses);
    EXPECT_TRUE(traj.is_valid());
    EXPECT_EQ(traj.num_poses(), 3);
}

// ============================================================================
// Position interpolation tests
// ============================================================================

TEST_F(Trajectory2DTest, PositionAtKnots) {
    Trajectory2D traj;
    traj.build(straight_poses);

    for (size_t i = 0; i < straight_poses.size(); i++) {
        Eigen::Vector2f pos = traj.position(static_cast<float>(i));
        EXPECT_NEAR(pos.x(), straight_poses[i].x(), 1e-4f);
        EXPECT_NEAR(pos.y(), straight_poses[i].y(), 1e-4f);
    }
}

TEST_F(Trajectory2DTest, PositionInterpolated) {
    Trajectory2D traj;
    traj.build(straight_poses);

    // Halfway between index 0 and 1
    Eigen::Vector2f pos = traj.position(0.5f);
    EXPECT_NEAR(pos.x(), 0.5f, 0.1f);
    EXPECT_NEAR(pos.y(), 0.0f, 0.1f);
}

TEST_F(Trajectory2DTest, PositionClampedAtBounds) {
    Trajectory2D traj;
    traj.build(straight_poses);

    // Before start
    Eigen::Vector2f pos_before = traj.position(-1.0f);
    EXPECT_NEAR(pos_before.x(), 0.0f, 1e-4f);

    // After end
    Eigen::Vector2f pos_after = traj.position(10.0f);
    EXPECT_NEAR(pos_after.x(), 3.0f, 1e-4f);
}

// ============================================================================
// Orientation tests
// ============================================================================

TEST_F(Trajectory2DTest, OrientationAtKnots) {
    Trajectory2D traj;
    std::vector<Eigen::Vector3f> poses = {
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(1, 0, 0.5f),
        Eigen::Vector3f(2, 0, 1.0f),
    };
    traj.build(poses);

    EXPECT_NEAR(traj.orientation(0.0f), 0.0f, 1e-4f);
    EXPECT_NEAR(traj.orientation(1.0f), 0.5f, 1e-4f);
    EXPECT_NEAR(traj.orientation(2.0f), 1.0f, 1e-4f);
}

TEST_F(Trajectory2DTest, OrientationNormalized) {
    Trajectory2D traj;
    std::vector<Eigen::Vector3f> poses = {
        Eigen::Vector3f(0, 0, 3.0f),
        Eigen::Vector3f(1, 0, -3.0f), // Wraps around
    };
    traj.build(poses);

    float theta = traj.orientation(0.5f);
    // Use small epsilon for floating-point comparison
    constexpr float epsilon = 1e-5f;
    EXPECT_TRUE(theta >= -core::PI - epsilon && theta <= core::PI + epsilon);
}

// ============================================================================
// Pose tests
// ============================================================================

TEST_F(Trajectory2DTest, PoseReturnsCorrectStruct) {
    Trajectory2D traj;
    traj.build(straight_poses);

    Pose2D pose = traj.pose(1.0f);
    EXPECT_NEAR(pose.x(), 1.0f, 1e-4f);
    EXPECT_NEAR(pose.y(), 0.0f, 1e-4f);
    EXPECT_NEAR(pose.theta(), 0.0f, 1e-4f);
}

TEST_F(Trajectory2DTest, PoseVectorMatchesPose) {
    Trajectory2D traj;
    traj.build(straight_poses);

    Pose2D pose = traj.pose(1.5f);
    Eigen::Vector3f vec = traj.pose_vector(1.5f);

    EXPECT_NEAR(pose.x(), vec.x(), 1e-6f);
    EXPECT_NEAR(pose.y(), vec.y(), 1e-6f);
    EXPECT_NEAR(pose.theta(), vec.z(), 1e-6f);
}

// ============================================================================
// Velocity tests
// ============================================================================

TEST_F(Trajectory2DTest, VelocityOnStraightLine) {
    Trajectory2D traj;
    traj.build(straight_poses);

    // On a straight line moving in +x direction, velocity should be (1, 0)
    Eigen::Vector2f vel = traj.velocity(1.5f);
    EXPECT_GT(vel.x(), 0.5f); // Moving in positive x
    EXPECT_NEAR(vel.y(), 0.0f, 0.1f);
}

TEST_F(Trajectory2DTest, AngularVelocityOnStraightLine) {
    Trajectory2D traj;
    traj.build(straight_poses);

    // No rotation on straight line
    float omega = traj.angular_velocity(1.5f);
    EXPECT_NEAR(omega, 0.0f, 0.1f);
}

TEST_F(Trajectory2DTest, SpeedConsistentWithVelocity) {
    Trajectory2D traj;
    traj.build(circular_poses);

    float t = 3.5f;
    Eigen::Vector2f vel = traj.velocity(t);
    float speed = traj.speed(t);

    EXPECT_NEAR(speed, vel.norm(), 1e-5f);
}

// ============================================================================
// Acceleration tests
// ============================================================================

TEST_F(Trajectory2DTest, AccelerationOnStraightLine) {
    Trajectory2D traj;
    traj.build(straight_poses);

    // Constant velocity means zero acceleration
    Eigen::Vector2f acc = traj.acceleration(1.5f);
    EXPECT_NEAR(acc.x(), 0.0f, 0.2f);
    EXPECT_NEAR(acc.y(), 0.0f, 0.2f);
}

TEST_F(Trajectory2DTest, AngularAccelerationOnStraightLine) {
    Trajectory2D traj;
    traj.build(straight_poses);

    float alpha = traj.angular_acceleration(1.5f);
    EXPECT_NEAR(alpha, 0.0f, 0.1f);
}

// ============================================================================
// Length and bounds tests
// ============================================================================

TEST_F(Trajectory2DTest, TotalLength) {
    Trajectory2D traj;
    traj.build(straight_poses);

    EXPECT_NEAR(traj.total_length(), 3.0f, 1e-5f);
    EXPECT_NEAR(traj.max_t(), 3.0f, 1e-5f);
}

TEST_F(Trajectory2DTest, InvalidTrajectoryReturnsZeros) {
    Trajectory2D traj;
    // Not built yet

    EXPECT_EQ(traj.position(0).x(), 0.0f);
    EXPECT_EQ(traj.position(0).y(), 0.0f);
    EXPECT_EQ(traj.orientation(0), 0.0f);
    EXPECT_EQ(traj.velocity(0).norm(), 0.0f);
    EXPECT_EQ(traj.speed(0), 0.0f);
    EXPECT_EQ(traj.total_length(), 0.0f);
}
