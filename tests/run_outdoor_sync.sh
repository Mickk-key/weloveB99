#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
mkdir -p build
g++ -std=c++14 -O2 -Wno-psabi -Isrc -Isim/native tests/outdoor_sync_tests.cpp -o build/outdoor_sync_tests -pthread $(pkg-config --cflags --libs opencv4)
./build/outdoor_sync_tests
for env in indoor outdoor outdoor_v2 sim; do
    ./build/race --env "$env" --modules all --check-config
done
