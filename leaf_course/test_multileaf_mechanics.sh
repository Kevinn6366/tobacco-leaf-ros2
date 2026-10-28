#!/usr/bin/env bash
# --replay: one verified three-leaf cycle; --full: three single + three multi.
# Mechanics only. No perception, batch feeding, grasp attachment or teleportation.
set -e
source /opt/ros/jazzy/setup.bash
set -u -o pipefail
course_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$course_dir"
mode="${1:---replay}"
case "$mode" in --replay|--full) ;; *) echo 'Usage: bash test_multileaf_mechanics.sh [--replay|--full]' >&2; exit 2;; esac
mkdir -p evidence
exec 9>evidence/mechanics.lock
flock -n 9 || { echo 'Another mechanics run is active' >&2; exit 1; }
trial_variant="cradle-run-$(date +%Y%m%d-%H%M%S)"
stage_dir="$(mktemp -d /tmp/leaf_cradle_run.XXXXXX)"
if [[ ! -f physics/contact-build/libleaf-contact-physics.so ]]; then
  bash physics/build_contact.sh
fi
python3 prepare_geometry_stage.py --output "$stage_dir/base"
(cd "$stage_dir/base" && python3 build_scene.py && python3 verify_leaf_geometry.py)
python3 experiments/multileaf_motion/prepare_cradle.py \
  --source "$stage_dir/base" --output "$stage_dir/cradle"
trial_args=(--variant "$trial_variant" --coupled-fingers \
  --geometry-root "$stage_dir/cradle" \
  --test-script "$course_dir/experiments/multileaf_motion/cradle.py")
if [[ "$mode" == --full ]]; then
  python3 -u run_contact_stage.py --count 1 --seeds 1 2 3 "${trial_args[@]}"
  python3 -u run_contact_stage.py --count 3 --seeds 1 2 3 "${trial_args[@]}"
else
  python3 -u run_contact_stage.py --count 3 --seeds 1 \
    --single-baseline "${LEAF_SINGLE_BASELINE:?Set LEAF_SINGLE_BASELINE to a successful --full trial directory name}" "${trial_args[@]}"
fi
printf 'Evidence: %s/evidence/geometry-stage/%s\n' "$course_dir" "$trial_variant"
