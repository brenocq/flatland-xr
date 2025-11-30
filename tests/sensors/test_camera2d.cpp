// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <gtest/gtest.h>
#include <sensors/camera2d.hpp>

using namespace sensors;

class Camera2DTest : public ::testing::Test {
  protected:
    void SetUp() override {
        camera.set_intrinsics(100, M_PI / 2); // 100px width, 90 degree FOV
        camera.set_noise_std(1.0f);
    }

    Camera2D camera;
};

// ============================================================================
// Intrinsics tests
// ============================================================================

TEST_F(Camera2DTest, SetIntrinsics) {
    Camera2D cam;
    cam.set_intrinsics(200, M_PI / 3); // 200px, 60 degree FOV

    EXPECT_EQ(cam.width(), 200);
    EXPECT_NEAR(cam.fov(), M_PI / 3, 1e-5f);
    EXPECT_NEAR(cam.principal_point(), 100.0f, 1e-5f);
}

TEST_F(Camera2DTest, SetWidthKeepsFOV) {
    float original_fov = camera.fov();
    camera.set_width(200);

    EXPECT_EQ(camera.width(), 200);
    EXPECT_NEAR(camera.fov(), original_fov, 1e-5f);
    EXPECT_NEAR(camera.principal_point(), 100.0f, 1e-5f);
}

TEST_F(Camera2DTest, SetFOVAdjustsFocalLength) {
    camera.set_fov(M_PI / 4); // 45 degrees
    EXPECT_NEAR(camera.fov(), M_PI / 4, 1e-5f);
}

TEST_F(Camera2DTest, FocalLengthCalculation) {
    // FOV = 2 * atan(width / (2 * fx))
    // For 90 degrees FOV and 100px width:
    // pi/2 = 2 * atan(100 / (2 * fx))
    // pi/4 = atan(50/fx)
    // tan(pi/4) = 50/fx
    // 1 = 50/fx
    // fx = 50
    EXPECT_NEAR(camera.focal_length(), 50.0f, 1e-4f);
}

TEST_F(Camera2DTest, IntrinsicsVector) {
    Eigen::Vector2f intrinsics = camera.intrinsics();
    EXPECT_NEAR(intrinsics.x(), camera.focal_length(), 1e-6f);
    EXPECT_NEAR(intrinsics.y(), camera.principal_point(), 1e-6f);
}

// ============================================================================
// Projection tests
// ============================================================================

TEST_F(Camera2DTest, ProjectLandmarkInFront) {
    // Camera at origin looking along +x
    Eigen::Vector3f pose(0, 0, 0);
    // Landmark directly in front
    Eigen::Vector2f landmark(5, 0);

    auto u = camera.project(pose, landmark);
    ASSERT_TRUE(u.has_value());
    EXPECT_NEAR(u.value(), 50.0f, 1e-4f); // Should be at center
}

TEST_F(Camera2DTest, ProjectLandmarkToLeft) {
    // Camera at origin looking along +x
    Eigen::Vector3f pose(0, 0, 0);
    // Landmark to the left (positive y in world)
    Eigen::Vector2f landmark(5, 2);

    auto u = camera.project(pose, landmark);
    ASSERT_TRUE(u.has_value());
    EXPECT_LT(u.value(), 50.0f); // Should be on left side of image
}

TEST_F(Camera2DTest, ProjectLandmarkToRight) {
    // Camera at origin looking along +x
    Eigen::Vector3f pose(0, 0, 0);
    // Landmark to the right (negative y in world)
    Eigen::Vector2f landmark(5, -2);

    auto u = camera.project(pose, landmark);
    ASSERT_TRUE(u.has_value());
    EXPECT_GT(u.value(), 50.0f); // Should be on right side of image
}

TEST_F(Camera2DTest, ProjectLandmarkBehindCamera) {
    Eigen::Vector3f pose(0, 0, 0);
    Eigen::Vector2f landmark(-5, 0); // Behind camera

    auto u = camera.project(pose, landmark);
    EXPECT_FALSE(u.has_value());
}

TEST_F(Camera2DTest, ProjectLandmarkOutsideFOV) {
    Eigen::Vector3f pose(0, 0, 0);
    // Way off to the side (outside 90 degree FOV)
    Eigen::Vector2f landmark(1, 10);

    auto u = camera.project(pose, landmark);
    EXPECT_FALSE(u.has_value());
}

TEST_F(Camera2DTest, ProjectWithRotatedCamera) {
    // Camera rotated 90 degrees (looking along +y)
    Eigen::Vector3f pose(0, 0, M_PI / 2);
    // Landmark along +y direction
    Eigen::Vector2f landmark(0, 5);

    auto u = camera.project(pose, landmark);
    ASSERT_TRUE(u.has_value());
    EXPECT_NEAR(u.value(), 50.0f, 1e-4f); // Should be at center
}

TEST_F(Camera2DTest, ProjectWithTranslatedCamera) {
    Eigen::Vector3f pose(3, 0, 0);
    Eigen::Vector2f landmark(8, 0); // 5 units in front

    auto u = camera.project(pose, landmark);
    ASSERT_TRUE(u.has_value());
    EXPECT_NEAR(u.value(), 50.0f, 1e-4f);
}

// ============================================================================
// Multiple landmarks projection
// ============================================================================

TEST_F(Camera2DTest, ProjectMultipleLandmarks) {
    Eigen::Vector3f pose(0, 0, 0);
    std::vector<Eigen::Vector2f> landmarks = {
        {5, 0},  // Center
        {5, 1},  // Left
        {5, -1}, // Right
        {-5, 0}, // Behind (should be filtered)
    };

    auto observations = camera.project_landmarks(pose, landmarks);
    EXPECT_EQ(observations.size(), 3u);

    // Check IDs are correct
    std::set<size_t> ids;
    for (const auto& obs : observations) {
        ids.insert(obs.landmark_id);
    }
    EXPECT_TRUE(ids.count(0) > 0);
    EXPECT_TRUE(ids.count(1) > 0);
    EXPECT_TRUE(ids.count(2) > 0);
    EXPECT_TRUE(ids.count(3) == 0);
}

TEST_F(Camera2DTest, ProjectNoVisibleLandmarks) {
    Eigen::Vector3f pose(0, 0, 0);
    std::vector<Eigen::Vector2f> landmarks = {
        {-5, 0}, // Behind
        {-5, 1}, // Behind
    };

    auto observations = camera.project_landmarks(pose, landmarks);
    EXPECT_TRUE(observations.empty());
}

// ============================================================================
// Measurement (noisy projection) tests
// ============================================================================

TEST_F(Camera2DTest, MeasureLandmark) {
    Eigen::Vector3f pose(0, 0, 0);
    Eigen::Vector2f landmark(5, 0);

    // Measure multiple times and check variance
    std::vector<float> measurements;
    for (int i = 0; i < 100; i++) {
        auto u = camera.measure(pose, landmark);
        ASSERT_TRUE(u.has_value());
        measurements.push_back(u.value());
    }

    // Calculate mean and std
    float mean = 0;
    for (float m : measurements)
        mean += m;
    mean /= measurements.size();

    float variance = 0;
    for (float m : measurements)
        variance += (m - mean) * (m - mean);
    variance /= measurements.size();
    float std = std::sqrtf(variance);

    // Mean should be close to true value
    EXPECT_NEAR(mean, 50.0f, 1.0f);
    // Std should be close to noise_std
    EXPECT_NEAR(std, 1.0f, 0.5f);
}

TEST_F(Camera2DTest, MeasureLandmarkBehindCamera) {
    Eigen::Vector3f pose(0, 0, 0);
    Eigen::Vector2f landmark(-5, 0);

    auto u = camera.measure(pose, landmark);
    EXPECT_FALSE(u.has_value());
}

TEST_F(Camera2DTest, MeasureMultipleLandmarks) {
    Eigen::Vector3f pose(0, 0, 0);
    std::vector<Eigen::Vector2f> landmarks = {
        {5, 0},
        {5, 1},
    };

    auto observations = camera.measure_landmarks(pose, landmarks);
    EXPECT_EQ(observations.size(), 2u);
}

// ============================================================================
// Noise configuration
// ============================================================================

TEST_F(Camera2DTest, SetNoiseStd) {
    camera.set_noise_std(2.0f);
    EXPECT_NEAR(camera.noise_std(), 2.0f, 1e-6f);
}

TEST_F(Camera2DTest, ZeroNoiseGivesExactProjection) {
    camera.set_noise_std(0.0f); // Zero noise
    Eigen::Vector3f pose(0, 0, 0);
    Eigen::Vector2f landmark(5, 0);

    auto gt = camera.project(pose, landmark);
    auto meas = camera.measure(pose, landmark);

    ASSERT_TRUE(gt.has_value());
    ASSERT_TRUE(meas.has_value());
    EXPECT_FLOAT_EQ(gt.value(), meas.value());
}
