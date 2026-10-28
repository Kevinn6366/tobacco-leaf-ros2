#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
root="$(cd -- "$(dirname -- "$0")/.." && pwd)"
export ROS_DOMAIN_ID=41 GZ_PARTITION=leaf_contact_v2
export LEAF_PHYSICS_ENGINE="$root/leaf_course/physics/contact-build/libleaf-contact-physics.so"
[[ -d "$root/leaf_course/live_mechanics" ]] || python3 "$root/scripts/prepare_demo.py"
pids=()
cleanup() { for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
python3 "$root/leaf_course/console_stereo.py" & pids+=("$!")
python3 "$root/leaf_course/console_telemetry.py" & pids+=("$!")
python3 "$root/leaf_course/mechanics_web.py" & pids+=("$!")
wait "${pids[2]}"
