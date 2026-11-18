#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
TEST_DIR="$(mktemp -d)"
trap 'rm -rf -- "$TEST_DIR"' EXIT
CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined)
"$CXX" "${FLAGS[@]}" -Wno-unused-variable \
 -I firmware/chassis/tests/host_stubs -I firmware/chassis/main/control \
 -I firmware/chassis/main/config -I firmware/chassis/components/pid_controller \
 firmware/chassis/tests/motion_sampling_test.cpp \
 firmware/chassis/main/control/motion_controller.cpp \
 firmware/chassis/main/control/differential_kinematics.cpp \
 firmware/chassis/components/pid_controller/pid_controller.cpp -o "$TEST_DIR/motion"
"$TEST_DIR/motion"
"$CXX" "${FLAGS[@]}" -I robot_ws/src/ldc_navigation/include \
 robot_ws/src/ldc_navigation/test/test_path_smoother.cpp -o "$TEST_DIR/smoother"
"$TEST_DIR/smoother"

"$CXX" "${FLAGS[@]}" -I firmware/chassis/main \
 -I firmware/chassis/main/tasks/network/microros \
 firmware/chassis/tests/sample_clock_test.cpp -o "$TEST_DIR/sample-clock"
"$TEST_DIR/sample-clock"

"$CXX" "${FLAGS[@]}" -I firmware/chassis/main \
 -I firmware/chassis/main/tasks/network/microros \
 firmware/chassis/tests/ultrasonic_publish_test.cpp -o "$TEST_DIR/ultrasonic-publish"
"$TEST_DIR/ultrasonic-publish"

"$CXX" "${FLAGS[@]}" -Wno-unused-parameter \
 -I firmware/chassis/tests/ultrasonic_stubs -I firmware/chassis/main \
 firmware/chassis/tests/ultrasonic_sampling_test.cpp \
 firmware/chassis/main/tasks/sensors/ultrasonic_task.cpp -o "$TEST_DIR/ultrasonic-sampling"
"$TEST_DIR/ultrasonic-sampling"

"$CXX" "${FLAGS[@]}" -Wno-unused-parameter \
 -I firmware/chassis/tests/sensor_stubs -I firmware/chassis/main \
 firmware/chassis/tests/imu_sampling_test.cpp \
 firmware/chassis/main/tasks/sensors/imu_task.cpp -o "$TEST_DIR/imu-sampling"
"$TEST_DIR/imu-sampling"

"$CXX" "${FLAGS[@]}" -I firmware/chassis/main -I firmware/chassis/main/config \
 -I firmware/chassis/main/control firmware/chassis/tests/motion_safety_test.cpp -o "$TEST_DIR/safety"
"$TEST_DIR/safety"
"$CXX" "${FLAGS[@]}" -Wno-unused-variable -pthread \
 -I firmware/chassis/tests/authority_stubs -I firmware/chassis/tests/host_stubs \
 -I firmware/chassis/main -I firmware/chassis/main/control -I firmware/chassis/main/core \
 -I firmware/chassis/main/config -I firmware/chassis/components/pid_controller \
 firmware/chassis/tests/motion_task_authority_test.cpp \
 firmware/chassis/main/core/motion_authority.cpp \
 firmware/chassis/main/tasks/control/motion_task.cpp \
 firmware/chassis/main/control/motion_controller.cpp \
 firmware/chassis/main/control/differential_kinematics.cpp \
 firmware/chassis/components/pid_controller/pid_controller.cpp -o "$TEST_DIR/task-authority"
"$TEST_DIR/task-authority"
"$CXX" "${FLAGS[@]}" -I firmware/chassis/main/tasks/network/provision \
 firmware/chassis/tests/bounded_body_test.cpp -o "$TEST_DIR/bounded-body"
"$TEST_DIR/bounded-body"
