#!/usr/bin/env bash
set -e
btth4_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$btth4_root/env.sh"
exec python3 "$btth4_root/runtime/stop_simulation.py"
