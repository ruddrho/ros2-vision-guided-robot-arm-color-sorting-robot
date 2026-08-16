#pragma once

#include "robot_model.hpp"

#include <vector>

namespace robot {

struct TrajectorySample {
    double time{0.0};
    JointVector position{};
    JointVector velocity{};
    JointVector acceleration{};
    Pose endEffector{};
};

class QuinticTrajectoryPlanner {
public:
    explicit QuinticTrajectoryPlanner(double sampleTime = 0.02);

    std::vector<TrajectorySample> planSegment(const RobotModel& robot,
                                              const JointVector& start,
                                              const JointVector& goal,
                                              double duration,
                                              double startTime = 0.0) const;

    std::vector<TrajectorySample> planWaypoints(
        const RobotModel& robot,
        const std::vector<JointVector>& waypoints,
        const std::vector<double>& segmentDurations) const;

    double sampleTime() const;

private:
    double sampleTime_;
};

}  // namespace robot
