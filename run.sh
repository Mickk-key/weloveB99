#!/bin/sh
set -eu
cd "$(dirname "$0")"
environment=
remaining=$#
while [ "$remaining" -gt 0 ]; do
  arg=$1; shift; remaining=$((remaining - 1))
  case "$arg" in
    --env-indoor|--env-outdoor)
      selected=${arg#--env-}
      if [ -n "$environment" ] && [ "$environment" != "$selected" ]; then
        echo 'Choose one environment' >&2; exit 2
      fi
      environment=$selected ;;
    *) set -- "$@" "$arg" ;;
  esac
done
[ -n "$environment" ] || { echo 'Specify --env-indoor or --env-outdoor, or use indoor/run.sh / outdoor/run.sh' >&2; exit 2; }
exec bash "$environment/run.sh" "$@"
