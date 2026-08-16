#include "robot_model.hpp"

#include <cmath>
#include <utility>

namespace robot {

RobotModel::RobotModel(std::array<DhLink, kDof> links,
                       std::array<JointLimit, kDof> limits,
                       std::string name)
    : links_(links), limits_(limits), name_(std::move(name)) {}

RobotModel RobotModel::educationalSixAxisArm() {
    const std::array<DhLink, kDof> links{{
        {0.0, kPi / 2.0, 0.18, 0.0},
        {0.32, 0.0, 0.0, 0.0},
        {0.28, 0.0, 0.0, 0.0},
        {0.0, kPi / 2.0, 0.22, 0.0},
        {0.0, -kPi / 2.0, 0.0, 0.0},
        {0.0, 0.0, 0.10, 0.0},
    }};

    const std::array<JointLimit, kDof> limits{{
        {-kPi, kPi},
        {-2.35, 2.35},
        {-2.35, 2.35},
        {-kPi, kPi},
        {-2.50, 2.50},
        {-kPi, kPi},
    }};

    return RobotModel(links, limits, "Educational 6-DOF Manipulator");
}

Mat4 RobotModel::forwardTransform(const JointVector& joints) const {
    Mat4 transform = identity4();
    for (std::size_t index = 0; index < kDof; ++index) {
        const auto& link = links_[index];
        transform = multiply(
            transform,
            dhTransform(link.a,
                        link.alpha,
                        link.d,
                        joints[index] + link.thetaOffset));
    }
    return transform;
}

Pose RobotModel::forward(const JointVector& joints) const {
    const Mat4 transform = forwardTransform(joints);
    return {translationOf(transform), rotationOf(transform)};
}

Jacobian RobotModel::geometricJacobian(const JointVector& joints) const {
    std::array<Vec3, kDof> origins{};
    std::array<Vec3, kDof> axes{};

    Mat4 transform = identity4();
    for (std::size_t index = 0; index < kDof; ++index) {
        origins[index] = translationOf(transform);
        axes[index] = axisZOf(transform);

        const auto& link = links_[index];
        transform = multiply(
            transform,
            dhTransform(link.a,
                        link.alpha,
                        link.d,
                        joints[index] + link.thetaOffset));
    }

    const Vec3 endEffector = translationOf(transform);
    Jacobian jacobian{};
    for (std::size_t column = 0; column < kDof; ++column) {
        const Vec3 linear = cross(axes[column], endEffector - origins[column]);
        jacobian[0][column] = linear.x;
        jacobian[1][column] = linear.y;
        jacobian[2][column] = linear.z;
        jacobian[3][column] = axes[column].x;
        jacobian[4][column] = axes[column].y;
        jacobian[5][column] = axes[column].z;
    }
    return jacobian;
}

JointVector RobotModel::clampToLimits(const JointVector& joints) const {
    JointVector clamped = joints;
    for (std::size_t index = 0; index < kDof; ++index) {
        clamped[index] =
            clamp(clamped[index], limits_[index].minimum, limits_[index].maximum);
    }
    return clamped;
}

bool RobotModel::withinLimits(const JointVector& joints, double tolerance) const {
    for (std::size_t index = 0; index < kDof; ++index) {
        if (joints[index] < limits_[index].minimum - tolerance ||
            joints[index] > limits_[index].maximum + tolerance) {
            return false;
        }
    }
    return true;
}

const std::array<DhLink, kDof>& RobotModel::links() const {
    return links_;
}

const std::array<JointLimit, kDof>& RobotModel::limits() const {
    return limits_;
}

const std::string& RobotModel::name() const {
    return name_;
}

Vector6 poseError(const Pose& current, const Pose& target) {
    const Vec3 positionDifference = target.position - current.position;
    const Mat3 rotationDifference =
        multiply(target.rotation, transpose(current.rotation));
    const Vec3 orientationDifference = rotationLog(rotationDifference);
    return {
        positionDifference.x,
        positionDifference.y,
        positionDifference.z,
        orientationDifference.x,
        orientationDifference.y,
        orientationDifference.z,
    };
}

double positionError(const Pose& current, const Pose& target) {
    return norm(target.position - current.position);
}

double orientationError(const Pose& current, const Pose& target) {
    const Mat3 rotationDifference =
        multiply(target.rotation, transpose(current.rotation));
    return norm(rotationLog(rotationDifference));
}

}  // namespace robot
