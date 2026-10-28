#pragma once
#include <array>
#include <cstddef>
#include <vector>
#include <utility>
#include <string>
#include <optional>

inline constexpr size_t NUM_JOINTS = 6;
inline constexpr size_t NUM_QUINTIC_COEFFICIENTS = 6;

/// @brief A single configuration along a trajectory including
/// a timestamp and arrays: position, velocity, and acceleration.
struct TrajectoryPoint {
    double time;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
};

/// @brief A segment bewtween two points in a trajectory with timestamps
/// for each waypoint and quintic time scaling coefficients
struct TrajectorySegment {
    double start_time;
    double end_time;
    std::array<std::array<double, NUM_QUINTIC_COEFFICIENTS>, NUM_JOINTS> coefficients;
};

/// @brief A time-scaled trajectory and segments along the trajectory
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    std::vector<TrajectorySegment> segments;
};

/// @brief Builds a quintic time scaled trajectory from a sparse, flat,
/// time-stamped list of via points.
///
/// For this robot which has 6 joints, each point is length 19:
/// (1 timestamp + position[6] + velocity[6] + acceleration[6]).
/// Therefore, a valid list of points is divisble by 19.
/// @param values flatten list of via points in the trajectory
std::pair<std::optional<Trajectory>, std::string> build_quintic_trajectory(const std::vector<double>& values);

/// @brief Finds the joint positions alonge the trajectory at a given time
/// @param trajectory Time-parametrized Trajectory
/// @param time Seconds from start of trajectory to sample at
std::vector<double> sample_trajectory(const Trajectory& trajectory, double time);

/// @brief Flattens vector to a 2D vector
template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T>>& input) {
    std::vector<T> result;
    size_t total_size = 0;
    for (const auto& inner : input) {
        total_size += inner.size();
    }
    result.reserve(total_size);
    for (const auto& inner : input) {
        result.insert(result.end(), inner.begin(), inner.end());
    }
    return result;
}

/// @brief Reshapes a flat vector to a 2d vector with the specified inner
template <typename T>
std::optional<std::vector<std::vector<T>>> unflatten(const std::vector<T>& input, size_t inner_size) {
    if (inner_size == 0 || (input.size() % inner_size) != 0) {
        return std::nullopt;
    }
    std::vector<std::vector<T>> result;

    for (std::size_t i = 0; i < input.size(); i += inner_size) {
        result.emplace_back(input.begin() + i, input.begin() + i + inner_size);
    }
    return result;
}
