#!/bin/sh
set -eu
cd "$(dirname "$0")"
exec ./build/race --config "$PWD/config/outdoor.yaml" "$@"
