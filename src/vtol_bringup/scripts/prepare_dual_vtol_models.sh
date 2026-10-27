#!/usr/bin/env bash
# Clone alti_transition_quad into per-vehicle models with distinct JSON FDM ports.
# Ensures Gazebo vtol_1 <-> SITL -I0:9002 and vtol_2 <-> SITL -I1:9012 with no cross-bind.
set -euo pipefail

SRC_RAW="${ALTITUDE_MODEL_SRC:-/home/uav/ardupilot_gazebo/models/alti_transition_quad}"
DST_BASE="${1:-/tmp/vtol_gz_models}"

if [[ ! -e "$SRC_RAW" ]]; then
  echo "[prepare_dual_vtol_models] missing source model: $SRC_RAW" >&2
  exit 1
fi

# Resolve symlinks so each vehicle gets a real private copy (not a shared link).
SRC="$(readlink -f "$SRC_RAW")"
if [[ ! -d "$SRC" ]]; then
  echo "[prepare_dual_vtol_models] source is not a directory: $SRC" >&2
  exit 1
fi

rm -rf "$DST_BASE"
mkdir -p "$DST_BASE"

prepare_one() {
  local name="$1"
  local port="$2"
  local dest="$DST_BASE/$name"

  cp -a "$SRC" "$dest"
  sed -i "s/name=\"alti_transition_quad\"/name=\"${name}\"/" "$dest/model.sdf"
  # Upstream model may ship with 9002 or 9012; force the instance port.
  sed -i -E "s#<fdm_port_in>[0-9]+</fdm_port_in>#<fdm_port_in>${port}</fdm_port_in>#" "$dest/model.sdf"
  sed -i "s#<name>Alti Transition QuadPlane</name>#<name>${name}</name>#" "$dest/model.config"

  if ! grep -q "<fdm_port_in>${port}</fdm_port_in>" "$dest/model.sdf"; then
    echo "[prepare_dual_vtol_models] failed to set fdm_port_in=${port} on ${name}" >&2
    exit 1
  fi
  # Guard against leftover duplicate plugin ports.
  local count
  count="$(grep -c "<fdm_port_in>" "$dest/model.sdf" || true)"
  if [[ "$count" -ne 1 ]]; then
    echo "[prepare_dual_vtol_models] expected 1 fdm_port_in in ${name}, found ${count}" >&2
    exit 1
  fi
  echo "[prepare_dual_vtol_models] ${name} -> JSON port ${port}"
}

prepare_one "vtol_1" "9002"
prepare_one "vtol_2" "9012"
echo "[prepare_dual_vtol_models] models ready under ${DST_BASE}"
