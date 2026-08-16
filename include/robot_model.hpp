#pragma once

#include "robot_math.hpp"

#include <array>
#include <string>

namespace robot {

struct DhLink {
    double a{0.0};
    double alpha{0.0};
    double d{0.0};
    double thetaOffset{0.0};
};

struct JointLimit {
    double minimum{-kPi};
    double maximum{kPi};
};

struct Pose {
    Vec3 position{};
    Mat3 rotation{identity3()};
};

using Jacobian = Matrix6;

class RobotModel {
public:
    RobotModel(std::array<DhLink, kDof> links,
               std::array<JointLimit, kDof> limits,
               std::string name);

    static RobotModel educationalSixAxisArm();

    Mat4 forwardTransform(const JointVector& joints) const;
    Pose forward(const JointVector& joints) const;
    Jacobian geometricJacobian(const JointVector& joints) const;
    JointVector clampToLimits(const JointVector& joints) const;
    bool withinLimits(const JointVector& joints, double tolerance = 1e-10) const;

    const std::array<DhLink, kDof>& links() const;
    const std::array<JointLimit, kDof>& limits() const;
    const std::string& name() const;

private:
    std::array<DhLink, kDof> links_{};
    std::array<JointLimit, kDof> limits_{};
    std::string name_;
};

Vector6 poseError(const Pose& current, const Pose& target);
double positionError(const Pose& current, const Pose& target);
double orientationError(const Pose& current, const Pose& target);

}  // namespace robot
