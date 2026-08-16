#pragma once

#include <array>
#include <cstddef>

namespace robot {

constexpr std::size_t kDof = 6;
constexpr double kPi = 3.14159265358979323846;

using JointVector = std::array<double, kDof>;
using Vector6 = std::array<double, 6>;
using Matrix6 = std::array<std::array<double, 6>, 6>;

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Mat3 {
    std::array<std::array<double, 3>, 3> data{};

    std::array<double, 3>& operator[](std::size_t index) {
        return data[index];
    }

    const std::array<double, 3>& operator[](std::size_t index) const {
        return data[index];
    }
};

struct Mat4 {
    std::array<std::array<double, 4>, 4> data{};
};

Vec3 operator+(const Vec3& lhs, const Vec3& rhs);
Vec3 operator-(const Vec3& lhs, const Vec3& rhs);
Vec3 operator*(double scalar, const Vec3& vector);
Vec3 operator*(const Vec3& vector, double scalar);
Vec3 operator/(const Vec3& vector, double scalar);

double dot(const Vec3& lhs, const Vec3& rhs);
Vec3 cross(const Vec3& lhs, const Vec3& rhs);
double norm(const Vec3& vector);

Mat3 identity3();
Mat4 identity4();
Mat3 transpose(const Mat3& matrix);
Mat3 multiply(const Mat3& lhs, const Mat3& rhs);
Mat4 multiply(const Mat4& lhs, const Mat4& rhs);
Vec3 multiply(const Mat3& matrix, const Vec3& vector);

Mat4 dhTransform(double a, double alpha, double d, double theta);
Mat3 rotationOf(const Mat4& transform);
Vec3 translationOf(const Mat4& transform);
Vec3 axisZOf(const Mat4& transform);
Vec3 rotationLog(const Mat3& rotation);

double clamp(double value, double minimum, double maximum);
double vectorNorm(const Vector6& vector);
bool solveLinearSystem(Matrix6 matrix, Vector6 rhs, Vector6& solution);

}  // namespace robot
