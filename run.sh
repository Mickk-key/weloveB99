#!/bin/sh
set -eu
cd "$(dirname "$0")"
env=
for arg do
 case "$arg" in --env-indoor) env=indoor;; --env-outdoor) env=outdoor;; esac
done
[ -n "$env" ] || { echo 'Select --env-indoor or --env-outdoor' >&2; exit 2; }
set -- "$@"
remaining=$#
while [ "$remaining" -gt 0 ]; do
 arg=$1; shift; remaining=$((remaining - 1))
 case "$arg" in --env-indoor|--env-outdoor) ;; *) set -- "$@" "$arg";; esac
done
exec ./build/race --env "$env" --config "$PWD/config/$env.yaml" "$@"
