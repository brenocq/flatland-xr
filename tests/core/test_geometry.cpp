// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 Breno Cunha Queiroz

#include <cmath>
#include <core/geometry.hpp>
#include <core/math.hpp>
#include <gtest/gtest.h>

using namespace core;

// ============================================================================
// segment_intersection tests
// ============================================================================

TEST(SegmentIntersection, CrossingSegments) {
    Eigen::Vector2f p1(0, 0), p2(2, 2);
    Eigen::Vector2f p3(0, 2), p4(2, 0);
    auto result = segment_intersection(p1, p2, p3, p4);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x(), 1.0f, 1e-5f);
    EXPECT_NEAR(result->y(), 1.0f, 1e-5f);
}

TEST(SegmentIntersection, ParallelSegments) {
    Eigen::Vector2f p1(0, 0), p2(2, 0);
    Eigen::Vector2f p3(0, 1), p4(2, 1);
    auto result = segment_intersection(p1, p2, p3, p4);
    EXPECT_FALSE(result.has_value());
}

TEST(SegmentIntersection, NonIntersectingSegments) {
    Eigen::Vector2f p1(0, 0), p2(1, 0);
    Eigen::Vector2f p3(2, 0), p4(3, 0);
    auto result = segment_intersection(p1, p2, p3, p4);
    EXPECT_FALSE(result.has_value());
}

TEST(SegmentIntersection, TouchingAtEndpoint) {
    Eigen::Vector2f p1(0, 0), p2(1, 1);
    Eigen::Vector2f p3(1, 1), p4(2, 0);
    auto result = segment_intersection(p1, p2, p3, p4);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x(), 1.0f, 1e-5f);
    EXPECT_NEAR(result->y(), 1.0f, 1e-5f);
}

TEST(SegmentIntersection, PerpendicularIntersection) {
    Eigen::Vector2f p1(-1, 0), p2(1, 0);
    Eigen::Vector2f p3(0, -1), p4(0, 1);
    auto result = segment_intersection(p1, p2, p3, p4);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x(), 0.0f, 1e-5f);
    EXPECT_NEAR(result->y(), 0.0f, 1e-5f);
}

// ============================================================================
// segments_intersect tests
// ============================================================================

TEST(SegmentsIntersect, ReturnsTrueWhenIntersecting) { EXPECT_TRUE(segments_intersect({0, 0}, {2, 2}, {0, 2}, {2, 0})); }

TEST(SegmentsIntersect, ReturnsFalseWhenNotIntersecting) { EXPECT_FALSE(segments_intersect({0, 0}, {1, 0}, {2, 0}, {3, 0})); }

// ============================================================================
// ray_segment_intersection tests
// ============================================================================

TEST(RaySegmentIntersection, RayHitsSegment) {
    Eigen::Vector2f origin(0, 0);
    Eigen::Vector2f dir(1, 0);
    Eigen::Vector2f seg_start(2, -1), seg_end(2, 1);
    auto result = ray_segment_intersection(origin, dir, seg_start, seg_end);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x(), 2.0f, 1e-5f);
    EXPECT_NEAR(result->y(), 0.0f, 1e-5f);
}

TEST(RaySegmentIntersection, RayMissesSegment) {
    Eigen::Vector2f origin(0, 0);
    Eigen::Vector2f dir(1, 0);
    Eigen::Vector2f seg_start(2, 1), seg_end(2, 2);
    auto result = ray_segment_intersection(origin, dir, seg_start, seg_end);
    EXPECT_FALSE(result.has_value());
}

TEST(RaySegmentIntersection, RayPointsAwayFromSegment) {
    Eigen::Vector2f origin(0, 0);
    Eigen::Vector2f dir(-1, 0);
    Eigen::Vector2f seg_start(2, -1), seg_end(2, 1);
    auto result = ray_segment_intersection(origin, dir, seg_start, seg_end);
    EXPECT_FALSE(result.has_value());
}

TEST(RaySegmentIntersection, ParallelRayAndSegment) {
    Eigen::Vector2f origin(0, 0);
    Eigen::Vector2f dir(1, 0);
    Eigen::Vector2f seg_start(0, 1), seg_end(2, 1);
    auto result = ray_segment_intersection(origin, dir, seg_start, seg_end);
    EXPECT_FALSE(result.has_value());
}

// ============================================================================
// point_to_segment_distance tests
// ============================================================================

TEST(PointToSegmentDistance, PointProjectsOntoSegment) {
    Eigen::Vector2f point(1, 1);
    Eigen::Vector2f seg_start(0, 0), seg_end(2, 0);
    float dist = point_to_segment_distance(point, seg_start, seg_end);
    EXPECT_NEAR(dist, 1.0f, 1e-5f);
}

TEST(PointToSegmentDistance, PointCloserToStart) {
    Eigen::Vector2f point(-1, 0);
    Eigen::Vector2f seg_start(0, 0), seg_end(2, 0);
    float dist = point_to_segment_distance(point, seg_start, seg_end);
    EXPECT_NEAR(dist, 1.0f, 1e-5f);
}

TEST(PointToSegmentDistance, PointCloserToEnd) {
    Eigen::Vector2f point(3, 0);
    Eigen::Vector2f seg_start(0, 0), seg_end(2, 0);
    float dist = point_to_segment_distance(point, seg_start, seg_end);
    EXPECT_NEAR(dist, 1.0f, 1e-5f);
}

TEST(PointToSegmentDistance, PointOnSegment) {
    Eigen::Vector2f point(1, 0);
    Eigen::Vector2f seg_start(0, 0), seg_end(2, 0);
    float dist = point_to_segment_distance(point, seg_start, seg_end);
    EXPECT_NEAR(dist, 0.0f, 1e-5f);
}

TEST(PointToSegmentDistance, DegenerateSegment) {
    Eigen::Vector2f point(1, 1);
    Eigen::Vector2f seg_start(0, 0), seg_end(0, 0);
    float dist = point_to_segment_distance(point, seg_start, seg_end);
    EXPECT_NEAR(dist, std::sqrt(2.0f), 1e-5f);
}

// ============================================================================
// normalize_angle tests
// ============================================================================

TEST(NormalizeAngle, AngleInRange) {
    EXPECT_NEAR(normalize_angle(0.5f), 0.5f, 1e-5f);
    EXPECT_NEAR(normalize_angle(-0.5f), -0.5f, 1e-5f);
}

TEST(NormalizeAngle, AngleAbovePi) {
    float angle = core::PI + 0.5f;
    float normalized = normalize_angle(angle);
    EXPECT_NEAR(normalized, -core::PI + 0.5f, 1e-5f);
}

TEST(NormalizeAngle, AngleBelowNegativePi) {
    float angle = -core::PI - 0.5f;
    float normalized = normalize_angle(angle);
    EXPECT_NEAR(normalized, core::PI - 0.5f, 1e-5f);
}

TEST(NormalizeAngle, MultipleTwoPi) {
    float angle = 4.0f * core::PI + 0.3f;
    float normalized = normalize_angle(angle);
    EXPECT_NEAR(normalized, 0.3f, 1e-5f);
}

TEST(NormalizeAngle, ExactlyPi) {
    // Pi should stay as pi (or close to it)
    float normalized = normalize_angle(core::PI);
    EXPECT_TRUE(std::abs(normalized) <= core::PI + 1e-5f);
}

// ============================================================================
// angle_difference tests
// ============================================================================

TEST(AngleDifference, SmallDifference) {
    EXPECT_NEAR(angle_difference(0.0f, 0.5f), 0.5f, 1e-5f);
    EXPECT_NEAR(angle_difference(0.5f, 0.0f), -0.5f, 1e-5f);
}

TEST(AngleDifference, WrapAroundPositive) {
    // From -3 to 3: the raw difference is 6, which wraps to 6-2pi ~ -0.283
    float diff = angle_difference(-3.0f, 3.0f);
    // The shortest path from -3 to 3 is actually the negative direction
    EXPECT_NEAR(std::abs(diff), 2.0f * core::PI - 6.0f, 1e-5f);
}

TEST(AngleDifference, WrapAroundNegative) {
    float diff = angle_difference(3.0f, -3.0f);
    // The shortest path from 3 to -3 is the positive direction
    EXPECT_NEAR(std::abs(diff), 2.0f * core::PI - 6.0f, 1e-5f);
}

TEST(AngleDifference, SameAngle) { EXPECT_NEAR(angle_difference(1.0f, 1.0f), 0.0f, 1e-5f); }

// ============================================================================
// unwrap_angles tests
// ============================================================================

TEST(UnwrapAngles, EmptyInput) {
    std::vector<float> empty;
    auto result = unwrap_angles(empty);
    EXPECT_TRUE(result.empty());
}

TEST(UnwrapAngles, SingleElement) {
    std::vector<float> single = {1.5f};
    auto result = unwrap_angles(single);
    ASSERT_EQ(result.size(), 1);
    EXPECT_NEAR(result[0], 1.5f, 1e-5f);
}

TEST(UnwrapAngles, NoWrapping) {
    std::vector<float> angles = {0.0f, 0.5f, 1.0f, 1.5f};
    auto result = unwrap_angles(angles);
    ASSERT_EQ(result.size(), 4);
    for (size_t i = 0; i < angles.size(); i++) {
        EXPECT_NEAR(result[i], angles[i], 1e-5f);
    }
}

TEST(UnwrapAngles, WrappingPositive) {
    // Angles that wrap from positive to negative
    std::vector<float> angles = {3.0f, -3.0f}; // Jump at pi boundary
    auto result = unwrap_angles(angles);
    ASSERT_EQ(result.size(), 2);
    EXPECT_NEAR(result[0], 3.0f, 1e-5f);
    // The difference should be small (not ~6)
    float diff = result[1] - result[0];
    EXPECT_TRUE(std::abs(diff) < core::PI + 0.1f);
}

TEST(UnwrapAngles, ContinuousRotation) {
    // Simulate continuous rotation
    std::vector<float> angles;
    for (int i = 0; i < 10; i++) {
        float angle = normalize_angle(i * 0.8f);
        angles.push_back(angle);
    }
    auto result = unwrap_angles(angles);
    // The unwrapped angles should be monotonically increasing
    for (size_t i = 1; i < result.size(); i++) {
        EXPECT_GT(result[i], result[i - 1] - 0.1f); // Allow small numerical errors
    }
}
