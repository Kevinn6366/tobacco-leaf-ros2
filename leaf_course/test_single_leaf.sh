#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
if [[ ! -f physics/contact-build/libleaf-contact-physics.so ]]; then
  bash physics/build_contact.sh
fi
python3 prepare_geometry_stage.py
(
 cd /tmp/leaf_geometry_v2
 python3 build_scene.py
 python3 verify_leaf_geometry.py
)
python3 run_contact_stage.py --count 1 --seeds 1 2 3 --variant "single-repeat-$(date +%Y%m%d-%H%M%S)"
