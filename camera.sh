#!/bin/sh
set -eu
cd "$(dirname "$0")"
exec ./build/race --camera-only "$@"
