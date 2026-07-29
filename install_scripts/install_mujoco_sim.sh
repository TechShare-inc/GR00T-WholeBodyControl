#!/usr/bin/env bash
# install_mujoco_sim.sh
# Minimal venv setup for running the MuJoCo simulator (run_sim_loop.py).
# Skips XRoboToolkit SDK and teleop dependencies that are NOT needed for sim.
# Based on install_pico.sh — see that script for the full teleop setup.
#
# Usage:  bash install_scripts/install_mujoco_sim.sh   (run from repo root)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 0. Print detected architecture ───────────────────────────────────────────
ARCH="$(uname -m)"
echo "[OK] Architecture: $ARCH"

# ── 1. Ensure uv is installed and available ──────────────────────────────────
if ! command -v uv &>/dev/null; then
    echo "[INFO] uv not found – installing via official installer …"
    curl -LsSf https://astral.sh/uv/install.sh | sh

    # Source the uv env so it's available in this session
    if [ -f "$HOME/.local/bin/env" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.local/bin/env"
    elif [ -f "$HOME/.cargo/env" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.cargo/env"
    else
        export PATH="$HOME/.local/bin:$PATH"
    fi

    # Verify uv is now reachable
    if ! command -v uv &>/dev/null; then
        echo "[ERROR] uv installation succeeded but binary not found on PATH."
        echo "        Please add ~/.local/bin (or ~/.cargo/bin) to your PATH and re-run."
        exit 1
    fi
fi
echo "[OK] uv $(uv --version)"

# ── 2. Install a uv-managed Python 3.10 (includes dev headers / Python.h) ────
echo "[INFO] Installing uv-managed Python 3.10 (includes development headers) …"
uv python install 3.10
MANAGED_PY="$(uv python find --no-project 3.10)"
echo "[OK] Using Python: $MANAGED_PY"

# ── 3. Clean previous venv (if any) ──────────────────────────────────────────
cd "$REPO_ROOT"
echo "[INFO] Removing old .venv_sim (if present) …"
rm -rf .venv_sim

# ── 4. Create venv & install sim extra ────────────────────────────────────────
echo "[INFO] Creating .venv_sim with uv-managed Python 3.10 …"
uv venv .venv_sim --python "$MANAGED_PY" --prompt gear_sonic_sim
# shellcheck disable=SC1091
source .venv_sim/bin/activate
echo "[INFO] Installing gear_sonic[sim] …"
uv pip install -e "gear_sonic[sim]"

# ── 5. Pull Git LFS objects for bundled CycloneDDS (gear_sonic_deploy) ────────
if command -v git-lfs &>/dev/null; then
    echo "[INFO] Pulling Git LFS objects in gear_sonic_deploy …"
    (cd "$REPO_ROOT/gear_sonic_deploy" && git lfs pull)
fi

# ── 6. Expose the bundled Unitree/CycloneDDS libraries to Python builds ──────
UNITREE_SDK2_DIR="$REPO_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2"
UNITREE_DDS_DIR="$UNITREE_SDK2_DIR/thirdparty"

case "$ARCH" in
    aarch64|arm64)
        UNITREE_DDS_LIB_DIR="$UNITREE_DDS_DIR/lib/aarch64"
        ;;
    x86_64|amd64)
        UNITREE_DDS_LIB_DIR="$UNITREE_DDS_DIR/lib/x86_64"
        ;;
    *)
        echo "[ERROR] Unsupported architecture for bundled CycloneDDS: $ARCH"
        exit 1
        ;;
esac

if [ ! -f "$UNITREE_DDS_LIB_DIR/libddsc.so" ] || [ ! -d "$UNITREE_DDS_DIR/include" ]; then
    echo "[ERROR] Bundled CycloneDDS files not found under:"
    echo "        $UNITREE_DDS_DIR"
    exit 1
fi

# The cyclonedds Python package expects CYCLONEDDS_HOME to look like a normal
# install prefix with include/, lib/, and bin/. The vendored Unitree copy keeps
# libraries in lib/<arch>, so create a venv-local prefix made of symlinks.
LOCAL_CYCLONEDDS_HOME="$REPO_ROOT/.venv_sim/cyclonedds"
mkdir -p "$LOCAL_CYCLONEDDS_HOME/bin" "$LOCAL_CYCLONEDDS_HOME/lib"
ln -sfn "$UNITREE_DDS_DIR/include" "$LOCAL_CYCLONEDDS_HOME/include"
ln -sfn "$UNITREE_DDS_LIB_DIR/libddsc.so" "$LOCAL_CYCLONEDDS_HOME/lib/libddsc.so"
if [ -f "$UNITREE_DDS_LIB_DIR/libddsc.so.0" ]; then
    ln -sfn "$UNITREE_DDS_LIB_DIR/libddsc.so.0" "$LOCAL_CYCLONEDDS_HOME/lib/libddsc.so.0"
fi
if [ -f "$UNITREE_DDS_LIB_DIR/libddscxx.so" ]; then
    ln -sfn "$UNITREE_DDS_LIB_DIR/libddscxx.so" "$LOCAL_CYCLONEDDS_HOME/lib/libddscxx.so"
fi
if [ -f "$UNITREE_DDS_LIB_DIR/libddscxx.so.0" ]; then
    ln -sfn "$UNITREE_DDS_LIB_DIR/libddscxx.so.0" "$LOCAL_CYCLONEDDS_HOME/lib/libddscxx.so.0"
fi

export CYCLONEDDS_HOME="$LOCAL_CYCLONEDDS_HOME"
export LD_LIBRARY_PATH="$LOCAL_CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="$LOCAL_CYCLONEDDS_HOME/lib:${LIBRARY_PATH:-}"
echo "[OK] Using bundled CycloneDDS: $CYCLONEDDS_HOME"

# Persist the runtime library path whenever this venv is activated later.
if ! grep -q "gear_sonic local CycloneDDS" .venv_sim/bin/activate; then
    cat >> .venv_sim/bin/activate <<EOF

# gear_sonic local CycloneDDS
export CYCLONEDDS_HOME="$LOCAL_CYCLONEDDS_HOME"
export LD_LIBRARY_PATH="\$CYCLONEDDS_HOME/lib:\${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="\$CYCLONEDDS_HOME/lib:\${LIBRARY_PATH:-}"
EOF
fi

# ── 7. Install unitree_sdk2_python (needed by the sim ↔ WBC bridge) ──────────
echo "[INFO] Installing unitree_sdk2_python …"
uv pip install -e external_dependencies/unitree_sdk2_python

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Setup complete!  Activate the venv with:"
echo ""
echo "    source .venv_sim/bin/activate"
echo ""
echo "  You should see (gear_sonic_sim) in your prompt."
echo ""
echo "  Then run the MuJoCo simulator with:"
echo "    python gear_sonic/scripts/run_sim_loop.py"
echo "══════════════════════════════════════════════════════════════"
