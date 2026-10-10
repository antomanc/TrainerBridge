#!/usr/bin/env bash
# Build and run the host regression tests. SANITIZE=1 adds AddressSanitizer + UBSan.
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
flags=(-std=c++17 -Wall -Wextra -Werror -Wno-unused-parameter -g)
if [[ "${SANITIZE:-0}" == 1 ]]; then
  flags+=(-fsanitize=address,undefined -fno-sanitize-recover=all)
fi
"${CXX:-g++}" "${flags[@]}" \
  -Itests/stubs -ITrainerBridge tests/firmware_test.cpp \
  TrainerBridge/{CommandQueue,PowerFilter,TrainerMatcher,VirtualSpeedModel,StatusLed}.cpp \
  -o "$build_dir/firmware_test"
"$build_dir/firmware_test"
