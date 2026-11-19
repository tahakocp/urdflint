#!/bin/sh
# Re-run the corpus numbers quoted in the README.
set -eu
dir=${1:-/tmp/urdf_files_dataset}
rev=81f4cda
[ -d "$dir" ] || git clone -q https://github.com/Daniella1/urdf_files_dataset "$dir"
git -C "$dir" checkout -q "$rev"
cargo build --release -q
time target/release/urdflint -q "$dir"
target/release/urdflint --format json "$dir" | grep -o '"code":"[EW][0-9]*"' | sort | uniq -c | sort -rn
