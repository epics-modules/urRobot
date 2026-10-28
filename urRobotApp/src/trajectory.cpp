#include "trajectory.hpp"
#include <cmath>
#include <algorithm>

/// For this robot which has 6 joints, each point is length 19:
/// (1 timestamp + position[6] + velocity[6] + acceleration[6])
constexpr size_t POINT_NUM_VALUES = 19;

std::pair<std::optional<Trajectory>, std::string> build_quintic_trajectory(const std::vector<double>& values) {

    if (values.size() < 2 * POINT_NUM_VALUES ||
        values.size() % POINT_NUM_VALUES != 0) {
        return {std::nullopt, "Invalid size of flat trajectory"};
    }

    auto trajectory_points = unflatten(values, POINT_NUM_VALUES);
    if (!trajectory_points) {
        return {std::nullopt, "Failed to reshape flat trajectory"};
    }

    Trajectory traj{};
    traj.points.reserve(trajectory_points->size());

    auto finite = [](const std::vector<double>& vec) -> bool {
        return std::all_of(
            vec.begin(),
            vec.end(),
            [](double value){
                return std::isfinite(value);
            });
    };

    double last_time = 0.0;
    for (auto it = trajectory_points->begin(); it != trajectory_points->end(); ++it) {
        const auto& row = *it;
        std::vector<double> pos(row.begin() + 1, row.begin() + 7);
        std::vector<double> vel(row.begin() + 7, row.begin() + 13);
        std::vector<double> acc(row.begin() + 13, row.begin() + 19);
        const double t = row[0];

        if (it == trajectory_points->begin()) {
            if (t != 0.0) {
                return {std::nullopt, "First timestamp must be 0.0"};
            }
        } else if (t <= last_time || !std::isfinite(t)) {
            return {std::nullopt, "Timestamps must be strictly increasing finite values"};
        }
        last_time = t;


        if (!finite(pos) || !finite(vel) || !finite(acc)) {
            return {std::nullopt, "Trajectory contains non-finite values"};
        }

        traj.points.push_back(TrajectoryPoint{
            t,
            pos, vel, acc
        });
    }

    traj.segments.reserve(traj.points.size() - 1);
    for (size_t point_index = 0; point_index + 1 < traj.points.size(); ++point_index) {
        const auto& start = traj.points[point_index];
        const auto& end = traj.points[point_index + 1];
        const double duration = end.time - start.time;

        TrajectorySegment segment{};
        segment.start_time = start.time;
        segment.end_time = end.time;

        for (size_t joint = 0; joint < NUM_JOINTS; ++joint) {
            auto& coefficients = segment.coefficients[joint];
            coefficients[0] = start.position[joint];
            coefficients[1] = duration * start.velocity[joint];
            coefficients[2] = 0.5 * duration * duration * start.acceleration[joint];

            const double position_error = end.position[joint] - (coefficients[0] + coefficients[1] + coefficients[2]);
            const double velocity_error = duration * end.velocity[joint] - (coefficients[1] + 2.0 * coefficients[2]);
            const double acceleration_error = duration * duration * end.acceleration[joint] - 2.0 * coefficients[2];

            coefficients[3] = 10.0 * position_error - 4.0 * velocity_error + 0.5 * acceleration_error;
            coefficients[4] = -15.0 * position_error + 7.0 * velocity_error - acceleration_error;
            coefficients[5] = 6.0 * position_error - 3.0 * velocity_error + 0.5 * acceleration_error;
        }

        traj.segments.push_back(segment);
    }

    return std::pair{std::move(traj), ""};
}

std::vector<double> sample_trajectory(const Trajectory& trajectory, double time) {

    // If the time is at or before the first point in the trajectory,
    // just return the first point
    if (time <= trajectory.points.front().time) {
        return trajectory.points.front().position;
    }

    // If the time is at or after the last point in the trajectory,
    // just return the last point
    if (time >= trajectory.points.back().time) {
        return trajectory.points.back().position;
    }

    // Find the point segment that the request time falls in
    const TrajectorySegment* segment = &trajectory.segments.front();
    for (const auto& candidate : trajectory.segments) {
        if (time <= candidate.end_time) {
            segment = &candidate;
            break;
        }
    }

    // convert absolute time from trajectory start to normalized segment time,
    // where at segment start, u = 0, and at segment end u = 1.
    // This is needed because coefficients were calculated for normalized time.
    const double u = (time - segment->start_time) / (segment->end_time - segment->start_time);

    // std::array<double, NUM_JOINTS> positions{};
    std::vector<double> positions(NUM_JOINTS);
    for (size_t joint = 0; joint < NUM_JOINTS; ++joint) {

        // Gets the six quintic coefficients for the current joint.
        // c[0] through c[5] represent:
        // q(u) = c0 + c1*u + c2*u² + c3*u³ + c4*u⁴ + c5*u⁵
        const auto& c = segment->coefficients[joint];

        // Evaluate the quintic using Horner’s method.
        // It is equivalent to:
        // positions[joint] =
            // c[0] +
            // c[1] * u +
            // c[2] * u * u +
            // c[3] * u * u * u +
            // c[4] * u * u * u * u +
            // c[5] * u * u * u * u * u
        positions[joint] = c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
    }

    return positions;
}
