#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
src="$PWD/gz-physics-7.8.0"
if [[ ! -d "$src/.git" ]]; then
 git clone --depth 1 --branch gz-physics7_7.8.0 https://github.com/gazebosim/gz-physics.git "$src"
fi
python3 patch_contact.py "$src"
cmake -S . -B contact-build -DENGINE_SOURCE="$src" -DCMAKE_BUILD_TYPE=Release
cmake --build contact-build -j2
