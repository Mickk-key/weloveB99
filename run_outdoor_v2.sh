#!/bin/sh
set -eu
cd "$(dirname "$0")"

# 驾驶时禁用预览，兼容之前带 --masks/--show 的命令；纯识别仍可显式开窗。
motion=0
for arg do
    if [ "$arg" = "--enable-motion" ]; then motion=1; fi
done
if [ "$motion" = 1 ]; then
    remaining=$#
    while [ "$remaining" -gt 0 ]; do
        arg=$1
        shift
        remaining=$((remaining - 1))
        case "$arg" in
            --show|--masks) ;;
            *) set -- "$@" "$arg" ;;
        esac
    done
    echo '[PREVIEW] disabled for driving'
fi
exec ./build/race --env outdoor_v2 --config "$PWD/config/outdoor_v2.yaml" "$@"
