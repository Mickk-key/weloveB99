#!/bin/bash
set -eu
cd "$(dirname "$0")"
# 兼容旧命令；独立目录请直接使用 indoor/run.sh 或 outdoor/run.sh。
environment=
args=()
for arg in "$@"; do
  case "$arg" in
    --env-indoor|--env-outdoor)
      next=${arg#--env-}
      if [ -n "$environment" ] && [ "$environment" != "$next" ]; then echo 'Choose one environment' >&2; exit 2; fi
      environment=$next ;;
    *) args+=("$arg") ;;
  esac
done
[ -n "$environment" ] || { echo 'Use: bash indoor/run.sh ... or bash outdoor/run.sh ... (legacy: --env-indoor/--env-outdoor)' >&2; exit 2; }
exec bash "$environment/run.sh" "${args[@]}"
