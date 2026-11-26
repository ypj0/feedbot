#!/bin/sh
# Apply the Cartographer <-> Ceres 2.2 compatibility patch.
set -eu

src_dir=${1:?usage: fix_ceres_2.2.sh /path/to/cartographer-source}
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
python3 "$script_dir/patch_cartographer_ceres.py" "$src_dir"
