#include "svg_plotter.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace robot {
namespace {

double mapValue(double value,
                double inputMinimum,
                double inputMaximum,
                double outputMinimum,
                double outputMaximum) {
    if (std::abs(inputMaximum - inputMinimum) < 1e-12) {
        return 0.5 * (outputMinimum + outputMaximum);
    }
    const double ratio =
        (value - inputMinimum) / (inputMaximum - inputMinimum);
    return outputMinimum + ratio * (outputMaximum - outputMinimum);
}

}  // namespace

void writeTrajectorySvg(const std::vector<TrajectorySample>& trajectory,
                        const std::vector<double>& targetTimes,
                        const std::string& outputPath) {
    if (trajectory.empty()) {
        throw std::invalid_argument("trajectory cannot be empty");
    }

    constexpr double width = 1200.0;
    constexpr double height = 760.0;
    constexpr double left = 90.0;
    constexpr double right = 40.0;
    constexpr double top = 70.0;
    constexpr double bottom = 80.0;
    constexpr double panelGap = 55.0;
    constexpr double panelHeight = (height - top - bottom - panelGap) / 2.0;

    const double timeMinimum = trajectory.front().time;
    const double timeMaximum = trajectory.back().time;

    double jointMaximum = 0.0;
    double speedMaximum = 0.0;
    for (const auto& sample : trajectory) {
        for (std::size_t joint = 0; joint < kDof; ++joint) {
            jointMaximum = std::max(jointMaximum, std::abs(sample.position[joint]));
            speedMaximum = std::max(speedMaximum, std::abs(sample.velocity[joint]));
        }
    }
    jointMaximum = std::max(0.25, 1.10 * jointMaximum);
    speedMaximum = std::max(0.10, 1.10 * speedMaximum);

    const std::array<std::string, kDof> colors{{
        "#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2"}};

    std::ofstream output(outputPath);
    if (!output) {
        throw std::runtime_error("failed to create SVG output");
    }

    output << std::fixed << std::setprecision(2);
    output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1200\" "
              "height=\"760\" viewBox=\"0 0 1200 760\">\n";
    output << "<rect width=\"1200\" height=\"760\" fill=\"#ffffff\"/>\n";
    output << "<text x=\"600\" y=\"34\" text-anchor=\"middle\" "
              "font-family=\"Arial\" font-size=\"24\" font-weight=\"700\">"
              "6-DOF Pick-and-Place Joint Trajectory</text>\n";

    const auto drawPanel = [&](double panelTop,
                               double valueMinimum,
                               double valueMaximum,
                               const std::string& title,
                               bool velocityPanel) {
        const double plotWidth = width - left - right;
        output << "<rect x=\"" << left << "\" y=\"" << panelTop
               << "\" width=\"" << plotWidth << "\" height=\"" << panelHeight
               << "\" fill=\"#f8fafc\" stroke=\"#94a3b8\"/>\n";
        output << "<text x=\"" << left << "\" y=\"" << panelTop - 12
               << "\" font-family=\"Arial\" font-size=\"17\" "
                  "font-weight=\"700\">"
               << title << "</text>\n";

        const double zeroY = mapValue(0.0,
                                      valueMinimum,
                                      valueMaximum,
                                      panelTop + panelHeight,
                                      panelTop);
        output << "<line x1=\"" << left << "\" y1=\"" << zeroY
               << "\" x2=\"" << left + plotWidth << "\" y2=\"" << zeroY
               << "\" stroke=\"#cbd5e1\" stroke-dasharray=\"5 5\"/>\n";

        for (double targetTime : targetTimes) {
            const double x = mapValue(targetTime,
                                      timeMinimum,
                                      timeMaximum,
                                      left,
                                      left + plotWidth);
            output << "<line x1=\"" << x << "\" y1=\"" << panelTop
                   << "\" x2=\"" << x << "\" y2=\"" << panelTop + panelHeight
                   << "\" stroke=\"#e2e8f0\" stroke-dasharray=\"3 5\"/>\n";
        }

        for (std::size_t joint = 0; joint < kDof; ++joint) {
            output << "<polyline fill=\"none\" stroke=\"" << colors[joint]
                   << "\" stroke-width=\"2.2\" points=\"";
            for (const auto& sample : trajectory) {
                const double x = mapValue(sample.time,
                                          timeMinimum,
                                          timeMaximum,
                                          left,
                                          left + plotWidth);
                const double value = velocityPanel ? sample.velocity[joint]
                                                   : sample.position[joint];
                const double y = mapValue(value,
                                          valueMinimum,
                                          valueMaximum,
                                          panelTop + panelHeight,
                                          panelTop);
                output << x << ',' << y << ' ';
            }
            output << "\"/>\n";
        }
    };

    drawPanel(top,
              -jointMaximum,
              jointMaximum,
              "Joint position (rad)",
              false);
    drawPanel(top + panelHeight + panelGap,
              -speedMaximum,
              speedMaximum,
              "Joint velocity (rad/s)",
              true);

    const double legendY = height - 34.0;
    for (std::size_t joint = 0; joint < kDof; ++joint) {
        const double x = 250.0 + 125.0 * static_cast<double>(joint);
        output << "<line x1=\"" << x << "\" y1=\"" << legendY
               << "\" x2=\"" << x + 24.0 << "\" y2=\"" << legendY
               << "\" stroke=\"" << colors[joint]
               << "\" stroke-width=\"3\"/>\n";
        output << "<text x=\"" << x + 30.0 << "\" y=\"" << legendY + 5.0
               << "\" font-family=\"Arial\" font-size=\"14\">J"
               << joint + 1 << "</text>\n";
    }
    output << "</svg>\n";
}

}  // namespace robot
