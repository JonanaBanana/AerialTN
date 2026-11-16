#!/bin/bash
# Stop the stereo IR encoded recording launch and verify shutdown.
# Uses SIGINT first so ros2 bag record finalizes the bag cleanly.
# SIGKILL is intentionally avoided to prevent bag corruption.

LAUNCH_PATTERN="record_stereo_ir_encoded_launch"
BAG_PATTERN="ros2.*bag.*record"

WAIT_SIGINT=8   # seconds to wait after SIGINT (bag finalization takes time)
WAIT_SIGTERM=4  # seconds to wait after SIGTERM

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

ok()   { echo -e "${GREEN}[OK]${NC}    $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; }

# ── 1. Check something is actually running ────────────────────────────────────
launch_pids=$(pgrep -f "$LAUNCH_PATTERN")
bag_pids=$(pgrep -f "$BAG_PATTERN")

if [ -z "$launch_pids" ] && [ -z "$bag_pids" ]; then
    warn "No recording processes found. Already stopped?"
    exit 0
fi

echo "Found processes:"
[ -n "$launch_pids" ] && echo "  launch : PID(s) $launch_pids"
[ -n "$bag_pids"    ] && echo "  bag    : PID(s) $bag_pids"
echo ""

# ── 2. Send SIGINT (clean shutdown — bag finalizes) ───────────────────────────
echo "Sending SIGINT (waiting ${WAIT_SIGINT}s for clean bag finalization)..."
pkill -SIGINT -f "$LAUNCH_PATTERN" 2>/dev/null
pkill -SIGINT -f "$BAG_PATTERN"    2>/dev/null

sleep "$WAIT_SIGINT"

# ── 3. Check if stopped ───────────────────────────────────────────────────────
if ! pgrep -f "$LAUNCH_PATTERN" > /dev/null && ! pgrep -f "$BAG_PATTERN" > /dev/null; then
    ok "All recording processes stopped cleanly after SIGINT."
else
    warn "Still running after SIGINT. Sending SIGTERM..."
    pkill -SIGTERM -f "$LAUNCH_PATTERN" 2>/dev/null
    pkill -SIGTERM -f "$BAG_PATTERN"    2>/dev/null

    sleep "$WAIT_SIGTERM"

    if ! pgrep -f "$LAUNCH_PATTERN" > /dev/null && ! pgrep -f "$BAG_PATTERN" > /dev/null; then
        ok "Processes stopped after SIGTERM."
        warn "Bag file may not be fully finalized — verify it before use."
    else
        err "Processes still running after SIGTERM."
        err "Remaining PIDs:"
        pgrep -a -f "$LAUNCH_PATTERN" | sed 's/^/    /'
        pgrep -a -f "$BAG_PATTERN"    | sed 's/^/    /'
        err "Do NOT use SIGKILL — the bag recorder must finalize on its own."
        err "Wait longer or investigate why the process is stuck."
        exit 1
    fi
fi

# ── 4. Verify via ros2 node list ─────────────────────────────────────────────
echo ""
echo "Checking remaining ROS2 nodes..."

remaining=$(ros2 node list 2>/dev/null)
if [ -z "$remaining" ]; then
    ok "No ROS2 nodes running."
else
    warn "These ROS2 nodes are still active (may be unrelated):"
    echo "$remaining" | sed 's/^/    /'
fi
