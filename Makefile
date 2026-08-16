CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion
CPPFLAGS ?= -Iinclude

BUILD_DIR := build
LIB_SOURCES := src/robot_math.cpp src/robot_model.cpp src/inverse_kinematics.cpp \
	src/trajectory_planner.cpp src/svg_plotter.cpp
LIB_OBJECTS := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(LIB_SOURCES))

.PHONY: all demo test run clean

all: demo test

demo: $(BUILD_DIR)/robot_arm_demo

test: $(BUILD_DIR)/robot_arm_tests
	./$(BUILD_DIR)/robot_arm_tests

run: $(BUILD_DIR)/robot_arm_demo
	./$(BUILD_DIR)/robot_arm_demo

$(BUILD_DIR)/robot_arm_demo: $(LIB_OBJECTS) $(BUILD_DIR)/main.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD_DIR)/robot_arm_tests: $(LIB_OBJECTS) $(BUILD_DIR)/test_main.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD_DIR)/%.o: src/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/test_main.o: tests/test_main.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
