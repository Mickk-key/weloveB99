#!/bin/sh
set -eu
cd "$(dirname "$0")"
mkdir -p build
g++ -std=c++14 -O2 -Wno-psabi -DRACE_PI src/main.cpp -o build/race.new $(pkg-config --cflags --libs opencv4) -lpigpio -pthread -lrt
mv -f build/race.new build/race
