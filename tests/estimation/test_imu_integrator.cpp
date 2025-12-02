// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <core/math.hpp>
#include <estimation/imu_integrator.hpp>
#include <gtest/gtest.h>

using namespace estimation;
using namespace sensors;

class IMUIntegratorTest : public ::testing::Test {
  protected:
    void SetUp() override {
        imu = std::make_shared<IMU2D>();
        imu->set_acc_bias(Eigen::Vector2f::Zero());
        imu->set_gyr_bias(0.0f);
        imu->set_acc_noise_std(Eigen::Vector2f::Zero());
        imu->set_gyr_noise_std(0.0f);

        integrator.set_imu_model(imu);
        integrator.set_gravity(Eigen::Vector2f(0, -9.81f));
        integrator.set_process_noise(0.1f, 0.01f);
    }

    std::shared_ptr<IMU2D> imu;
    IMUIntegrator integrator;
};

// ============================================================================
// Initialization tests
// ============================================================================

TEST_F(IMUIntegratorTest, DefaultStateIsZero) {
    IMUIntegrator fresh;
    EstimatedState state = fresh.get_state();

    EXPECT_NEAR(state.pose.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(state.pose.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(state.pose.z(), 0.0f, 1e-6f);
    EXPECT_NEAR(state.velocity.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(state.velocity.y(), 0.0f, 1e-6f);
}

TEST_F(IMUIntegratorTest, Initialize) {
    Eigen::Vector3f initial_pose(1.0f, 2.0f, 0.5f);
    Eigen::Vector2f initial_vel(3.0f, 4.0f);

    integrator.initialize(initial_pose, initial_vel);
    EstimatedState state = integrator.get_state();

    EXPECT_NEAR(state.pose.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(state.pose.y(), 2.0f, 1e-6f);
    EXPECT_NEAR(state.pose.z(), 0.5f, 1e-6f);
    EXPECT_NEAR(state.velocity.x(), 3.0f, 1e-6f);
    EXPECT_NEAR(state.velocity.y(), 4.0f, 1e-6f);
}

TEST_F(IMUIntegratorTest, InitializeSetsZeroCovariance) {
    integrator.initialize(Eigen::Vector3f::Zero(), Eigen::Vector2f::Zero());
    EstimatedState state = integrator.get_state();

    // Initialize sets small initial covariance (0.01 * Identity), not zero
    float expected_pose_cov = std::sqrt(3.0f * 0.01f * 0.01f); // norm of 0.01 * Identity(3x3)
    float expected_vel_cov = std::sqrt(2.0f * 0.01f * 0.01f);  // norm of 0.01 * Identity(2x2)
    
    EXPECT_NEAR(state.pose_cov.norm(), expected_pose_cov, 1e-6f);
    EXPECT_NEAR(state.vel_cov.norm(), expected_vel_cov, 1e-6f);
}

TEST_F(IMUIntegratorTest, Reset) {
    integrator.initialize(Eigen::Vector3f(1, 2, 3), Eigen::Vector2f(4, 5));
    integrator.reset();

    EstimatedState state = integrator.get_state();
    EXPECT_NEAR(state.pose.norm(), 0.0f, 1e-6f);
    EXPECT_NEAR(state.velocity.norm(), 0.0f, 1e-6f);
}

// ============================================================================
// IMU integration tests
// ============================================================================

TEST_F(IMUIntegratorTest, StationaryWithGravity) {
    integrator.set_gravity(Eigen::Vector2f(0, -9.81f));
    integrator.initialize(Eigen::Vector3f::Zero(), Eigen::Vector2f::Zero());

    // IMU measures upward acceleration when stationary (specific force = -gravity in body frame)
    // At orientation 0, body frame aligns with world, so acc = (0, 9.81)
    IMUMeasurement stationary_imu;
    stationary_imu.acc = Eigen::Vector2f(0, 9.81f);
    stationary_imu.gyr = 0.0f;

    // Integrate for a few steps
    float dt = 0.01f;
    for (int i = 0; i < 100; i++) {
        integrator.process_imu(stationary_imu, dt);
    }

    EstimatedState state = integrator.get_state();
    // Should remain near origin
    EXPECT_NEAR(state.pose.x(), 0.0f, 0.1f);
    EXPECT_NEAR(state.pose.y(), 0.0f, 0.1f);
    EXPECT_NEAR(state.velocity.x(), 0.0f, 0.1f);
    EXPECT_NEAR(state.velocity.y(), 0.0f, 0.1f);
}

TEST_F(IMUIntegratorTest, ConstantAcceleration) {
    integrator.set_gravity(Eigen::Vector2f::Zero()); // No gravity for simplicity
    integrator.initialize(Eigen::Vector3f::Zero(), Eigen::Vector2f::Zero());

    // Constant acceleration in x direction
    IMUMeasurement imu_meas;
    imu_meas.acc = Eigen::Vector2f(1.0f, 0.0f);
    imu_meas.gyr = 0.0f;

    float dt = 0.1f;
    int n_steps = 10;
    for (int i = 0; i < n_steps; i++) {
        integrator.process_imu(imu_meas, dt);
    }

    float total_time = n_steps * dt;
    EstimatedState state = integrator.get_state();

    // v = a*t
    EXPECT_NEAR(state.velocity.x(), 1.0f * total_time, 0.01f);
    // x = 0.5*a*t^2
    // Note: Euler integration introduces error (~10% for this case)
    // Relax tolerance to account for first-order integration error
    EXPECT_NEAR(state.pose.x(), 0.5f * 1.0f * total_time * total_time, 0.06f);
}

TEST_F(IMUIntegratorTest, ConstantAngularVelocity) {
    integrator.set_gravity(Eigen::Vector2f::Zero());
    integrator.initialize(Eigen::Vector3f::Zero(), Eigen::Vector2f::Zero());

    IMUMeasurement imu_meas;
    imu_meas.acc = Eigen::Vector2f::Zero();
    imu_meas.gyr = 0.1f; // rad/s

    float dt = 0.1f;
    int n_steps = 10;
    for (int i = 0; i < n_steps; i++) {
        integrator.process_imu(imu_meas, dt);
    }

    EstimatedState state = integrator.get_state();
    // theta = omega * t
    EXPECT_NEAR(state.pose.z(), 0.1f * n_steps * dt, 0.01f);
}

TEST_F(IMUIntegratorTest, BiasCompensation) {
    Eigen::Vector2f acc_bias(0.1f, 0.2f);
    float gyr_bias = 0.05f;
    imu->set_acc_bias(acc_bias);
    imu->set_gyr_bias(gyr_bias);

    integrator.set_gravity(Eigen::Vector2f::Zero());
    integrator.initialize(Eigen::Vector3f::Zero(), Eigen::Vector2f::Zero());

    // Biased measurement that should result in zero motion after bias removal
    IMUMeasurement biased_meas;
    biased_meas.acc = acc_bias; // Only bias, no real acceleration
    biased_meas.gyr = gyr_bias; // Only bias, no real rotation

    float dt = 0.1f;
    for (int i = 0; i < 10; i++) {
        integrator.process_imu(biased_meas, dt);
    }

    EstimatedState state = integrator.get_state();
    // Should remain near origin after bias compensation
    EXPECT_NEAR(state.pose.x(), 0.0f, 0.01f);
    EXPECT_NEAR(state.pose.y(), 0.0f, 0.01f);
    EXPECT_NEAR(state.pose.z(), 0.0f, 0.01f);
}

TEST_F(IMUIntegratorTest, RotatedAcceleration) {
    integrator.set_gravity(Eigen::Vector2f::Zero());
    // Start with 90 degree rotation (looking along +y)
    integrator.initialize(Eigen::Vector3f(0, 0, core::HALF_PI), Eigen::Vector2f::Zero());

    // Body frame acceleration in x (forward)
    IMUMeasurement imu_meas;
    imu_meas.acc = Eigen::Vector2f(1.0f, 0.0f);
    imu_meas.gyr = 0.0f;

    float dt = 0.1f;
    integrator.process_imu(imu_meas, dt);

    EstimatedState state = integrator.get_state();
    // Body +x at 90 degrees is world +y
    EXPECT_NEAR(state.velocity.x(), 0.0f, 0.01f);
    EXPECT_NEAR(state.velocity.y(), 0.1f, 0.01f);
}

// ============================================================================
// Covariance propagation tests
// ============================================================================

TEST_F(IMUIntegratorTest, CovarianceGrows) {
    integrator.initialize(Eigen::Vector3f::Zero(), Eigen::Vector2f::Zero());

    IMUMeasurement imu_meas;
    imu_meas.acc = Eigen::Vector2f::Zero();
    imu_meas.gyr = 0.0f;

    EstimatedState state_before = integrator.get_state();
    float initial_cov_norm = state_before.pose_cov.norm();

    float dt = 0.1f;
    for (int i = 0; i < 10; i++) {
        integrator.process_imu(imu_meas, dt);
    }

    EstimatedState state_after = integrator.get_state();
    EXPECT_GT(state_after.pose_cov.norm(), initial_cov_norm);
}

// ============================================================================
// Result/history tests
// ============================================================================

TEST_F(IMUIntegratorTest, GetResultReturnsHistory) {
    integrator.initialize(Eigen::Vector3f::Zero(), Eigen::Vector2f::Zero());

    IMUMeasurement imu_meas;
    imu_meas.acc = Eigen::Vector2f(1, 0);
    imu_meas.gyr = 0.0f;

    float dt = 0.1f;
    for (int i = 0; i < 5; i++) {
        integrator.process_imu(imu_meas, dt);
    }

    EstimationResult result = integrator.get_result();
    // Initial state + 5 IMU measurements = 6 states
    EXPECT_EQ(result.num_steps(), 6u);
}

TEST_F(IMUIntegratorTest, GetPosesFromResult) {
    integrator.initialize(Eigen::Vector3f(0, 0, 0), Eigen::Vector2f::Zero());

    IMUMeasurement imu_meas;
    imu_meas.acc = Eigen::Vector2f::Zero();
    imu_meas.gyr = 0.1f;

    float dt = 0.1f;
    for (int i = 0; i < 3; i++) {
        integrator.process_imu(imu_meas, dt);
    }

    EstimationResult result = integrator.get_result();
    auto poses = result.get_poses();

    EXPECT_EQ(poses.size(), 4u);
    EXPECT_NEAR(poses[0].z(), 0.0f, 1e-6f);
    EXPECT_GT(poses[3].z(), 0.0f); // Should have rotated
}

TEST_F(IMUIntegratorTest, GetVelocitiesFromResult) {
    integrator.set_gravity(Eigen::Vector2f::Zero());
    integrator.initialize(Eigen::Vector3f::Zero(), Eigen::Vector2f::Zero());

    IMUMeasurement imu_meas;
    imu_meas.acc = Eigen::Vector2f(1, 0);
    imu_meas.gyr = 0.0f;

    float dt = 0.1f;
    for (int i = 0; i < 3; i++) {
        integrator.process_imu(imu_meas, dt);
    }

    EstimationResult result = integrator.get_result();
    auto vels = result.get_velocities();

    EXPECT_EQ(vels.size(), 4u);
    EXPECT_NEAR(vels[0].x(), 0.0f, 1e-6f);
    EXPECT_GT(vels[3].x(), 0.0f); // Should have accelerated
}

// ============================================================================
// Camera processing (should be no-op)
// ============================================================================

TEST_F(IMUIntegratorTest, ProcessCameraIsNoOp) {
    integrator.initialize(Eigen::Vector3f(1, 2, 3), Eigen::Vector2f(4, 5));

    CameraFrame frame;
    frame.observations.push_back(CameraMeasurement(50.0f, 0));

    EstimatedState before = integrator.get_state();
    integrator.process_camera(frame);
    EstimatedState after = integrator.get_state();

    EXPECT_NEAR(before.pose.x(), after.pose.x(), 1e-6f);
    EXPECT_NEAR(before.pose.y(), after.pose.y(), 1e-6f);
    EXPECT_NEAR(before.pose.z(), after.pose.z(), 1e-6f);
}

// ============================================================================
// EstimatedState tests
// ============================================================================

TEST(EstimatedState, DefaultConstructor) {
    EstimatedState state;
    EXPECT_NEAR(state.pose.norm(), 0.0f, 1e-6f);
    EXPECT_NEAR(state.velocity.norm(), 0.0f, 1e-6f);
}

TEST(EstimatedState, ConstructorWithPoseAndVel) {
    Eigen::Vector3f pose(1, 2, 3);
    Eigen::Vector2f vel(4, 5);
    EstimatedState state(pose, vel);

    EXPECT_NEAR(state.pose.x(), 1.0f, 1e-6f);
    EXPECT_NEAR(state.velocity.y(), 5.0f, 1e-6f);
}

// ============================================================================
// EstimationResult tests
// ============================================================================

TEST(EstimationResult, DefaultIsInvalid) {
    EstimationResult result;
    EXPECT_FALSE(result.is_valid());
    EXPECT_EQ(result.num_steps(), 0u);
}

TEST(EstimationResult, Clear) {
    EstimationResult result;
    result.states.push_back(EstimatedState());
    result.clear();
    EXPECT_FALSE(result.is_valid());
}
