#!/bin/bash
set -euo pipefail

[[ $# -eq 1 ]] || { echo "usage: $0 NEW_RESULTS_DIRECTORY" >&2; exit 2; }
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
results=$(realpath -m "$1")
[[ ! -e "$results" ]] || { echo "results directory already exists: $results" >&2; exit 2; }

exec systemd-run --collect --unit=feedbot-base --property=Type=simple --property=Restart=no \
    --property=TimeoutStopSec=180 /usr/bin/python3 -B "$script_dir/session.py" "$results"
