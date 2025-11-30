// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <core/types.hpp>
#include <gtest/gtest.h>

using namespace core;

// ============================================================================
// Pose2D tests
// ============================================================================

TEST(Pose2D, DefaultConstructor) {
    Pose2D pose;
    EXPECT_NEAR(pose.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.theta(), 0.0f, 1e-6f);
}

TEST(Pose2D, ConstructorWithValues) {
    Pose2D pose(1.0f, 2.0f, 0.5f);
    EXPECT_NEAR(pose.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(pose.y(), 2.0f, 1e-6f);
    EXPECT_NEAR(pose.theta(), 0.5f, 1e-6f);
}

TEST(Pose2D, ConstructorWithVector2f) {
    Eigen::Vector2f pos(3.0f, 4.0f);
    Pose2D pose(pos, 1.0f);
    EXPECT_NEAR(pose.x(), 3.0f, 1e-6f);
    EXPECT_NEAR(pose.y(), 4.0f, 1e-6f);
    EXPECT_NEAR(pose.theta(), 1.0f, 1e-6f);
}

TEST(Pose2D, ConstructorFromVector3f) {
    Eigen::Vector3f v(5.0f, 6.0f, 0.7f);
    Pose2D pose(v);
    EXPECT_NEAR(pose.x(), 5.0f, 1e-6f);
    EXPECT_NEAR(pose.y(), 6.0f, 1e-6f);
    EXPECT_NEAR(pose.theta(), 0.7f, 1e-6f);
}

TEST(Pose2D, ToVector) {
    Pose2D pose(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f v = pose.to_vector();
    EXPECT_NEAR(v.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(v.y(), 2.0f, 1e-6f);
    EXPECT_NEAR(v.z(), 3.0f, 1e-6f);
}

TEST(Pose2D, RotationMatrixBodyToWorldAtZero) {
    Pose2D pose(0, 0, 0);
    Eigen::Matrix2f R = pose.R_body_to_world();
    EXPECT_NEAR(R(0, 0), 1.0f, 1e-6f);
    EXPECT_NEAR(R(0, 1), 0.0f, 1e-6f);
    EXPECT_NEAR(R(1, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(R(1, 1), 1.0f, 1e-6f);
}

TEST(Pose2D, RotationMatrixBodyToWorldAt90Degrees) {
    Pose2D pose(0, 0, M_PI / 2);
    Eigen::Matrix2f R = pose.R_body_to_world();
    EXPECT_NEAR(R(0, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(R(0, 1), -1.0f, 1e-6f);
    EXPECT_NEAR(R(1, 0), 1.0f, 1e-6f);
    EXPECT_NEAR(R(1, 1), 0.0f, 1e-6f);
}

TEST(Pose2D, RotationMatrixWorldToBodyIsTranspose) {
    Pose2D pose(0, 0, 0.7f);
    Eigen::Matrix2f R_bw = pose.R_body_to_world();
    Eigen::Matrix2f R_wb = pose.R_world_to_body();
    EXPECT_NEAR((R_bw * R_wb - Eigen::Matrix2f::Identity()).norm(), 0.0f, 1e-6f);
}

TEST(Pose2D, RotationTransformsVector) {
    Pose2D pose(0, 0, M_PI / 2);
    Eigen::Vector2f body_vec(1, 0); // Forward in body frame
    Eigen::Vector2f world_vec = pose.R_body_to_world() * body_vec;
    EXPECT_NEAR(world_vec.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(world_vec.y(), 1.0f, 1e-6f);
}

// ============================================================================
// Velocity2D tests
// ============================================================================

TEST(Velocity2D, DefaultConstructor) {
    Velocity2D vel;
    EXPECT_NEAR(vel.vx(), 0.0f, 1e-6f);
    EXPECT_NEAR(vel.vy(), 0.0f, 1e-6f);
    EXPECT_NEAR(vel.omega(), 0.0f, 1e-6f);
}

TEST(Velocity2D, ConstructorWithValues) {
    Velocity2D vel(1.0f, 2.0f, 0.5f);
    EXPECT_NEAR(vel.vx(), 1.0f, 1e-6f);
    EXPECT_NEAR(vel.vy(), 2.0f, 1e-6f);
    EXPECT_NEAR(vel.omega(), 0.5f, 1e-6f);
}

TEST(Velocity2D, ConstructorWithVector2f) {
    Eigen::Vector2f lin(3.0f, 4.0f);
    Velocity2D vel(lin, 1.0f);
    EXPECT_NEAR(vel.vx(), 3.0f, 1e-6f);
    EXPECT_NEAR(vel.vy(), 4.0f, 1e-6f);
    EXPECT_NEAR(vel.omega(), 1.0f, 1e-6f);
}

// ============================================================================
// Acceleration2D tests
// ============================================================================

TEST(Acceleration2D, DefaultConstructor) {
    Acceleration2D acc;
    EXPECT_NEAR(acc.ax(), 0.0f, 1e-6f);
    EXPECT_NEAR(acc.ay(), 0.0f, 1e-6f);
    EXPECT_NEAR(acc.alpha(), 0.0f, 1e-6f);
}

TEST(Acceleration2D, ConstructorWithValues) {
    Acceleration2D acc(1.0f, 2.0f, 0.5f);
    EXPECT_NEAR(acc.ax(), 1.0f, 1e-6f);
    EXPECT_NEAR(acc.ay(), 2.0f, 1e-6f);
    EXPECT_NEAR(acc.alpha(), 0.5f, 1e-6f);
}

TEST(Acceleration2D, ConstructorWithVector2f) {
    Eigen::Vector2f lin(3.0f, 4.0f);
    Acceleration2D acc(lin, 1.0f);
    EXPECT_NEAR(acc.ax(), 3.0f, 1e-6f);
    EXPECT_NEAR(acc.ay(), 4.0f, 1e-6f);
    EXPECT_NEAR(acc.alpha(), 1.0f, 1e-6f);
}

// ============================================================================
// State2D tests
// ============================================================================

TEST(State2D, DefaultConstructor) {
    State2D state;
    EXPECT_NEAR(state.pose.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(state.velocity.vx(), 0.0f, 1e-6f);
}

TEST(State2D, ConstructorWithPoseAndVelocity) {
    Pose2D pose(1, 2, 0.5f);
    Velocity2D vel(3, 4, 0.1f);
    State2D state(pose, vel);
    EXPECT_NEAR(state.pose.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(state.velocity.vx(), 3.0f, 1e-6f);
}

// ============================================================================
// Landmark tests
// ============================================================================

TEST(Landmark, DefaultConstructor) {
    Landmark lm;
    EXPECT_NEAR(lm.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(lm.y(), 0.0f, 1e-6f);
    EXPECT_EQ(lm.id, 0u);
}

TEST(Landmark, ConstructorWithValues) {
    Landmark lm(1.0f, 2.0f, 5);
    EXPECT_NEAR(lm.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(lm.y(), 2.0f, 1e-6f);
    EXPECT_EQ(lm.id, 5u);
}

TEST(Landmark, ConstructorWithVector2f) {
    Eigen::Vector2f pos(3.0f, 4.0f);
    Landmark lm(pos, 10);
    EXPECT_NEAR(lm.x(), 3.0f, 1e-6f);
    EXPECT_NEAR(lm.y(), 4.0f, 1e-6f);
    EXPECT_EQ(lm.id, 10u);
}

// ============================================================================
// Wall tests
// ============================================================================

TEST(Wall, DefaultConstructor) {
    Wall wall;
    EXPECT_TRUE(wall.empty());
    EXPECT_EQ(wall.num_segments(), 0u);
}

TEST(Wall, ConstructorWithPoints) {
    std::vector<Eigen::Vector2f> pts = {
        {0, 0},
        {1, 0},
        {1, 1},
    };
    Wall wall(pts);
    EXPECT_FALSE(wall.empty());
    EXPECT_EQ(wall.points.size(), 3u);
    EXPECT_EQ(wall.num_segments(), 2u);
}

TEST(Wall, SinglePointNoSegments) {
    Wall wall;
    wall.points.push_back(Eigen::Vector2f(0, 0));
    EXPECT_FALSE(wall.empty());
    EXPECT_EQ(wall.num_segments(), 0u);
}
