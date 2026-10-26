#include <stdexcept>
#include "trajectory.hpp"

Trajectory build_trajectory(const std::vector<double>& values) {

    if (values.size() < FLAT_POINT_LENGTH * 2) {
        throw std::invalid_argument("Sparse flat trajectory is invalid length");
    }

    auto trajectory_points = unflatten(values, FLAT_POINT_LENGTH);
    Trajectory traj;
    traj.points.reserve(trajectory_points.size());

    for (const auto& row : trajectory_points) {
        std::vector<double> pos(row.begin() + 1, row.begin() + 7);
        std::vector<double> vel(row.begin() + 7, row.begin() + 13);
        std::vector<double> acc(row.begin() + 13, row.begin() + 19);
        traj.points.push_back(TrajectoryPoint{
            row[0],
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

    return traj;
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
