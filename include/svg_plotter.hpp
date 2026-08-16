#pragma once

#include "trajectory_planner.hpp"

#include <string>
#include <vector>

namespace robot {

void writeTrajectorySvg(const std::vector<TrajectorySample>& trajectory,
                        const std::vector<double>& targetTimes,
                        const std::string& outputPath);

}  // namespace robot
