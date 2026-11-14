#!/bin/sh
set -eu
cd "$(dirname "$0")"
mkdir -p build
g++ -std=c++14 -O2 -Isim/native src/main.cpp -o build/race.new $(pkg-config --cflags --libs opencv4) -pthread
mv -f build/race.new build/race
