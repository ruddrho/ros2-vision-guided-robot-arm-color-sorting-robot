#include "trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {

QuinticTrajectoryPlanner::QuinticTrajectoryPlanner(double sampleTime)
    : sampleTime_(sampleTime) {
    if (sampleTime_ <= 0.0) {
        throw std::invalid_argument("sample time must be positive");
    }
}

std::vector<TrajectorySample> QuinticTrajectoryPlanner::planSegment(
    const RobotModel& robot,
    const JointVector& start,
    const JointVector& goal,
    double duration,
    double startTime) const {
    if (duration <= 0.0) {
        throw std::invalid_argument("segment duration must be positive");
    }

    const std::size_t steps =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(duration / sampleTime_)));
    std::vector<TrajectorySample> samples;
    samples.reserve(steps + 1);

    for (std::size_t step = 0; step <= steps; ++step) {
        const double localTime = std::min(
            duration,
            static_cast<double>(step) * duration / static_cast<double>(steps));
        const double tau = localTime / duration;
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double tau4 = tau3 * tau;
        const double tau5 = tau4 * tau;

        const double blend = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
        const double blendRate =
            (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / duration;
        const double blendAcceleration =
            (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) /
            (duration * duration);

        TrajectorySample sample{};
        sample.time = startTime + localTime;
        for (std::size_t joint = 0; joint < kDof; ++joint) {
            const double displacement = goal[joint] - start[joint];
            sample.position[joint] = start[joint] + blend * displacement;
            sample.velocity[joint] = blendRate * displacement;
            sample.acceleration[joint] = blendAcceleration * displacement;
        }
        sample.endEffector = robot.forward(sample.position);
        samples.push_back(sample);
    }
    return samples;
}

std::vector<TrajectorySample> QuinticTrajectoryPlanner::planWaypoints(
    const RobotModel& robot,
    const std::vector<JointVector>& waypoints,
    const std::vector<double>& segmentDurations) const {
    if (waypoints.size() < 2) {
        throw std::invalid_argument("at least two waypoints are required");
    }
    if (segmentDurations.size() + 1 != waypoints.size()) {
        throw std::invalid_argument(
            "segment duration count must be one less than waypoint count");
    }

    std::vector<TrajectorySample> completeTrajectory;
    double startTime = 0.0;
    for (std::size_t segment = 0; segment < segmentDurations.size(); ++segment) {
        auto segmentSamples = planSegment(robot,
                                          waypoints[segment],
                                          waypoints[segment + 1],
                                          segmentDurations[segment],
                                          startTime);
        if (!completeTrajectory.empty() && !segmentSamples.empty()) {
            segmentSamples.erase(segmentSamples.begin());
        }
        completeTrajectory.insert(completeTrajectory.end(),
                                  segmentSamples.begin(),
                                  segmentSamples.end());
        startTime += segmentDurations[segment];
    }
    return completeTrajectory;
}

double QuinticTrajectoryPlanner::sampleTime() const {
    return sampleTime_;
}

}  // namespace robot
