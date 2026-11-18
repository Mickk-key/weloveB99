#!/bin/sh
set -eu
cd "$(dirname "$0")"
bash indoor/build_pi.sh
bash outdoor/build_pi.sh
