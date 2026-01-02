#!/bin/bash
set -e

# =============================
# Tunables (mapping-friendly)
# =============================
V=0.22           # m/s  (slow enough for 2–3 Hz scan)
W=0.30           # rad/s (arc turns; radius ~= V/W ~= 0.73 m)
ARC90=5.236      # seconds for 90 deg at 0.30 rad/s  (pi/2 / W)
PUB_RATE=10      # Hz command publishing (keeps Gazebo happy)

# Big sleeps to ensure scan+slam catch up (2 Hz -> 2s ~= 4 scans)
PAUSE_BETWEEN=2.0   # seconds

# Optional: small initial settle
SETTLE=2.0

# Optional: tiny start alignment forward (useful if you spawn very close to wall)
START_ALIGN_SECONDS=2.0

# =============================
# Geometry for YOUR 7x7 layout
# =============================
# Warehouse half-size = 3.5
# Obstacles (your reminder file):
#   shelves around x=1.6, y=0 and y=-1.4
#   deliveries around x=-2.6, y=0 and y=-1.4
#
# So we avoid hugging LEFT wall (deliveries are near it).
# Outer loop is a rounded rectangle that stays away from x=-2.6 line.
#
# OUTER loop (rounded rectangle):
#   x_left  = -1.9
#   x_right =  2.7
#   y_bot   = -2.7
#   y_top   =  2.7
#
# With radius r~0.73, straight distances are:
#   dx_outer = (x_right - x_left) - 2r  ~= (4.6) - 1.46 = 3.14 m
#   dy_outer = (y_top - y_bot)   - 2r  ~= (5.4) - 1.46 = 3.94 m
DX_OUTER=14.3   # seconds (3.14m / 0.22)
DY_OUTER=17.9   # seconds (3.94m / 0.22)

# INNER loop (rounded rectangle) more centered:
#   x_left  = -1.2
#   x_right =  2.0
#   y_bot   = -2.0
#   y_top   =  2.0
#
# Straight distances:
#   dx_inner ~= (3.2) - 1.46 = 1.74 m -> 7.9 s
#   dy_inner ~= (4.0) - 1.46 = 2.54 m -> 11.6 s
DX_INNER=7.9
DY_INNER=11.6

# Transition between loops: drift inward a bit without spinning
# (a gentle arc + short straight)
TRANS_ARC=2.6    # half of 90deg (about 45deg)
TRANS_FWD=3.0    # seconds forward to shift inward

# =============================
# Helpers
# =============================
pub_for() {
  local v="$1"
  local w="$2"
  local t="$3"

  # publish continuously for t seconds
  ros2 topic pub -r ${PUB_RATE} /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: ${v}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${w}}}" >/dev/null 2>&1 &
  local pid=$!
  sleep "${t}" || true
  kill "${pid}" >/dev/null 2>&1 || true
}

stop_and_wait() {
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once >/dev/null 2>&1 || true
  sleep "${PAUSE_BETWEEN}" || true
}

cleanup() {
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[auto_scan] settle ${SETTLE}s..."
sleep "${SETTLE}"

# Tiny forward nudge to get clear of the spawn wall area
if (( $(echo "${START_ALIGN_SECONDS} > 0" | bc -l) )); then
  echo "[auto_scan] start-align forward ${START_ALIGN_SECONDS}s..."
  pub_for "${V}" 0.0 "${START_ALIGN_SECONDS}"
  stop_and_wait
fi

# =========================================================
# OUTER LOOP (clockwise): +X, arc-right, -Y, arc-right, -X, arc-right, +Y, arc-right
# =========================================================
echo "[auto_scan] OUTER loop start..."

# Top edge (east)
pub_for "${V}" 0.0 "${DX_OUTER}"
stop_and_wait

# Corner: arc-right (east -> south)
pub_for "${V}" "-${W}" "${ARC90}"
stop_and_wait

# Right edge (south)
pub_for "${V}" 0.0 "${DY_OUTER}"
stop_and_wait

# Corner: arc-right (south -> west)
pub_for "${V}" "-${W}" "${ARC90}"
stop_and_wait

# Bottom edge (west)
pub_for "${V}" 0.0 "${DX_OUTER}"
stop_and_wait

# Corner: arc-right (west -> north)
pub_for "${V}" "-${W}" "${ARC90}"
stop_and_wait

# Left-ish edge (north)
pub_for "${V}" 0.0 "${DY_OUTER}"
stop_and_wait

# Corner: arc-right (north -> east) closes the loop
pub_for "${V}" "-${W}" "${ARC90}"
stop_and_wait

echo "[auto_scan] OUTER loop done (back near start)."

# =========================================================
# TRANSITION inward (no spin): gentle arc + straight
# =========================================================
echo "[auto_scan] Transition inward..."
pub_for "${V}" "-${W}" "${TRANS_ARC}"
stop_and_wait
pub_for "${V}" 0.0 "${TRANS_FWD}"
stop_and_wait
pub_for "${V}" "${W}" "${TRANS_ARC}"   # bend back to roughly original heading
stop_and_wait

# =========================================================
# INNER LOOP (clockwise) smaller rectangle
# =========================================================
echo "[auto_scan] INNER loop start..."

pub_for "${V}" 0.0 "${DX_INNER}"
stop_and_wait

pub_for "${V}" "-${W}" "${ARC90}"
stop_and_wait

pub_for "${V}" 0.0 "${DY_INNER}"
stop_and_wait

pub_for "${V}" "-${W}" "${ARC90}"
stop_and_wait

pub_for "${V}" 0.0 "${DX_INNER}"
stop_and_wait

pub_for "${V}" "-${W}" "${ARC90}"
stop_and_wait

pub_for "${V}" 0.0 "${DY_INNER}"
stop_and_wait

pub_for "${V}" "-${W}" "${ARC90}"
stop_and_wait

echo "[auto_scan] INNER loop done."

# Final stop
cleanup
echo "[auto_scan] finished."
