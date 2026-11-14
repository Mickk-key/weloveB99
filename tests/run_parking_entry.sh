#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build
for name in parking_entry_tests parking_edge_tests parking_smooth_tests qr_tests indoor_s_tests fixed_maneuver_tests; do
    g++ -std=c++14 -O2 -Wno-psabi -Isrc -Isim/native "tests/$name.cpp" -o "build/$name" -pthread $(pkg-config --cflags --libs opencv4)
    "build/$name"
done
