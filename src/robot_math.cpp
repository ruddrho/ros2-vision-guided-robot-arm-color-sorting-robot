#include "robot_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot {

Vec3 operator+(const Vec3& lhs, const Vec3& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(double scalar, const Vec3& vector) {
    return {scalar * vector.x, scalar * vector.y, scalar * vector.z};
}

Vec3 operator*(const Vec3& vector, double scalar) {
    return scalar * vector;
}

Vec3 operator/(const Vec3& vector, double scalar) {
    return {vector.x / scalar, vector.y / scalar, vector.z / scalar};
}

double dot(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 cross(const Vec3& lhs, const Vec3& rhs) {
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

double norm(const Vec3& vector) {
    return std::sqrt(dot(vector, vector));
}

Mat3 identity3() {
    Mat3 matrix{};
    for (std::size_t index = 0; index < 3; ++index) {
        matrix.data[index][index] = 1.0;
    }
    return matrix;
}

Mat4 identity4() {
    Mat4 matrix{};
    for (std::size_t index = 0; index < 4; ++index) {
        matrix.data[index][index] = 1.0;
    }
    return matrix;
}

Mat3 transpose(const Mat3& matrix) {
    Mat3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result.data[row][column] = matrix.data[column][row];
        }
    }
    return result;
}

Mat3 multiply(const Mat3& lhs, const Mat3& rhs) {
    Mat3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t index = 0; index < 3; ++index) {
                result.data[row][column] +=
                    lhs.data[row][index] * rhs.data[index][column];
            }
        }
    }
    return result;
}

Mat4 multiply(const Mat4& lhs, const Mat4& rhs) {
    Mat4 result{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t index = 0; index < 4; ++index) {
                result.data[row][column] +=
                    lhs.data[row][index] * rhs.data[index][column];
            }
        }
    }
    return result;
}

Vec3 multiply(const Mat3& matrix, const Vec3& vector) {
    return {
        matrix.data[0][0] * vector.x + matrix.data[0][1] * vector.y +
            matrix.data[0][2] * vector.z,
        matrix.data[1][0] * vector.x + matrix.data[1][1] * vector.y +
            matrix.data[1][2] * vector.z,
        matrix.data[2][0] * vector.x + matrix.data[2][1] * vector.y +
            matrix.data[2][2] * vector.z,
    };
}

Mat4 dhTransform(double a, double alpha, double d, double theta) {
    const double cosTheta = std::cos(theta);
    const double sinTheta = std::sin(theta);
    const double cosAlpha = std::cos(alpha);
    const double sinAlpha = std::sin(alpha);

    Mat4 transform = identity4();
    transform.data[0] = {
        cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta};
    transform.data[1] = {
        sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta};
    transform.data[2] = {0.0, sinAlpha, cosAlpha, d};
    transform.data[3] = {0.0, 0.0, 0.0, 1.0};
    return transform;
}

Mat3 rotationOf(const Mat4& transform) {
    Mat3 rotation{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            rotation.data[row][column] = transform.data[row][column];
        }
    }
    return rotation;
}

Vec3 translationOf(const Mat4& transform) {
    return {
        transform.data[0][3], transform.data[1][3], transform.data[2][3]};
}

Vec3 axisZOf(const Mat4& transform) {
    return {
        transform.data[0][2], transform.data[1][2], transform.data[2][2]};
}

Vec3 rotationLog(const Mat3& rotation) {
    const double trace = rotation.data[0][0] + rotation.data[1][1] +
                         rotation.data[2][2];
    const double cosAngle = clamp(0.5 * (trace - 1.0), -1.0, 1.0);
    const double angle = std::acos(cosAngle);

    const Vec3 skewVector{
        rotation.data[2][1] - rotation.data[1][2],
        rotation.data[0][2] - rotation.data[2][0],
        rotation.data[1][0] - rotation.data[0][1],
    };

    if (angle < 1e-8) {
        return 0.5 * skewVector;
    }

    const double sinAngle = std::sin(angle);
    if (std::abs(sinAngle) < 1e-7) {
        Vec3 axis{
            std::sqrt(std::max(0.0, 0.5 * (rotation.data[0][0] + 1.0))),
            std::sqrt(std::max(0.0, 0.5 * (rotation.data[1][1] + 1.0))),
            std::sqrt(std::max(0.0, 0.5 * (rotation.data[2][2] + 1.0))),
        };
        if (skewVector.x < 0.0) {
            axis.x = -axis.x;
        }
        if (skewVector.y < 0.0) {
            axis.y = -axis.y;
        }
        if (skewVector.z < 0.0) {
            axis.z = -axis.z;
        }
        const double axisNorm = norm(axis);
        return axisNorm > 1e-9 ? angle * (axis / axisNorm) : Vec3{};
    }

    return (angle / (2.0 * sinAngle)) * skewVector;
}

double clamp(double value, double minimum, double maximum) {
    return std::min(std::max(value, minimum), maximum);
}

double vectorNorm(const Vector6& vector) {
    double squaredNorm = 0.0;
    for (double value : vector) {
        squaredNorm += value * value;
    }
    return std::sqrt(squaredNorm);
}

bool solveLinearSystem(Matrix6 matrix, Vector6 rhs, Vector6& solution) {
    constexpr double kPivotTolerance = 1e-12;

    for (std::size_t pivot = 0; pivot < 6; ++pivot) {
        std::size_t bestRow = pivot;
        double bestMagnitude = std::abs(matrix[pivot][pivot]);
        for (std::size_t row = pivot + 1; row < 6; ++row) {
            const double magnitude = std::abs(matrix[row][pivot]);
            if (magnitude > bestMagnitude) {
                bestMagnitude = magnitude;
                bestRow = row;
            }
        }

        if (bestMagnitude < kPivotTolerance || !std::isfinite(bestMagnitude)) {
            return false;
        }

        if (bestRow != pivot) {
            std::swap(matrix[pivot], matrix[bestRow]);
            std::swap(rhs[pivot], rhs[bestRow]);
        }

        const double pivotValue = matrix[pivot][pivot];
        for (std::size_t column = pivot; column < 6; ++column) {
            matrix[pivot][column] /= pivotValue;
        }
        rhs[pivot] /= pivotValue;

        for (std::size_t row = 0; row < 6; ++row) {
            if (row == pivot) {
                continue;
            }
            const double factor = matrix[row][pivot];
            for (std::size_t column = pivot; column < 6; ++column) {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    solution = rhs;
    return true;
}

}  // namespace robot
