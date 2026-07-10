#!/usr/bin/env bash
# Non-interactive SONIC WBC runtime launcher — for use by systemd / sonicctl.
#
# This script performs environment/interface resolution and execs the
# already-built binary. It does NOT install packages, build, invoke sudo,
# or ask for confirmation. Those steps belong in provisioning/bootstrap.
#
# Usage:
#   scripts/run_noninteractive.sh <interface> [--checkpoint PATH] [--obs-config PATH]
#                                   [--planner PATH] [--motion-data PATH]
#                                   [--input-type TYPE] [--output-type TYPE]
#                                   [--zmq-host HOST] [--zmq-port PORT] [...]
#
# All options after <interface> are forwarded directly to g1_deploy_onnx_ref.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(dirname "$SCRIPT_DIR")"

# ---------------------------------------------------------------------------
# Interface resolution (extracted from deploy.sh's resolve_interface)
# ---------------------------------------------------------------------------
resolve_iface() {
  local mode="$1"

  case "$mode" in
    sim)
      echo "lo"          # loopback for MuJoCo
      return
      ;;
    real)
      # Auto-detect 192.168.123.x (Unitree G1 default subnet)
      local iface
      iface="$(ip -4 addr show 2>/dev/null | grep -oP '192\.168\.123\.\d+' | head -1 || true)"
      if [[ -z "$iface" ]]; then
        echo "[run_noninteractive] ERROR: no 192.168.123.x interface found for 'real' target" >&2
        exit 1
      fi
      local ifname
      ifname="$(ip -4 addr show to "$iface" 2>/dev/null | awk '/inet/ {print $NF}' | head -1)"
      echo "${ifname:-$iface}"
      return
      ;;
    *)
      # Pass through: could be interface name or IP
      echo "$mode"
      return
      ;;
  esac
}

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <sim|real|interface> [binary options...]" >&2
  exit 2
fi

INTERFACE_MODE="$1"
shift

TARGET_IFACE="$(resolve_iface "$INTERFACE_MODE")"

# ---------------------------------------------------------------------------
# Locate the binary
# ---------------------------------------------------------------------------
BINARY="$DEPLOY_DIR/target/release/g1_deploy_onnx_ref"
if [[ ! -x "$BINARY" ]]; then
  echo "[run_noninteractive] ERROR: binary not found at $BINARY" >&2
  echo "  Build first with: cd $DEPLOY_DIR && ./deploy.sh sim" >&2
  exit 1
fi

echo "[run_noninteractive] interface=$TARGET_IFACE (from '$INTERFACE_MODE')"
echo "[run_noninteractive] binary=$BINARY"

exec "$BINARY" "$TARGET_IFACE" "$@"
