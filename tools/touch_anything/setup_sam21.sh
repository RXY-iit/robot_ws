#!/usr/bin/env bash
# Reproduce the local, CPU-only SAM 2.1 dependency used by Touch Anything.

set -euo pipefail

TOUCH_WS="${TOUCH_WS:-/home/matsunaga-h/robot_ws}"
SAM_REPO="$TOUCH_WS/third_party/sam2"
SAM_DEPS="$TOUCH_WS/third_party/sam2_runtime_deps"
CHECKPOINT_DIR="$TOUCH_WS/models/sam2"
CHECKPOINT="$CHECKPOINT_DIR/sam2.1_hiera_tiny.pt"
SAM_COMMIT="2b90b9f5ceec907a1c18123530e92e794ad901a4b5"
CHECKPOINT_SHA256="7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69"

mkdir -p "$TOUCH_WS/third_party" "$CHECKPOINT_DIR"
touch "$TOUCH_WS/third_party/COLCON_IGNORE"

if [[ ! -d "$SAM_REPO/.git" ]]; then
  git clone --no-checkout https://github.com/facebookresearch/sam2.git "$SAM_REPO"
  git -C "$SAM_REPO" fetch --depth 1 origin "$SAM_COMMIT"
  git -C "$SAM_REPO" checkout --detach "$SAM_COMMIT"
fi

actual_commit="$(git -C "$SAM_REPO" rev-parse HEAD)"
if [[ "$actual_commit" != "$SAM_COMMIT" ]]; then
  echo "SAM source mismatch: expected $SAM_COMMIT, observed $actual_commit" >&2
  echo "Use the audited commit before running Touch Anything." >&2
  exit 1
fi

/usr/bin/python3 -m pip install \
  --target "$SAM_DEPS" \
  'hydra-core>=1.3.2,<1.4' \
  'iopath>=0.1.10,<0.2'

if [[ ! -f "$CHECKPOINT" ]]; then
  curl -fL --retry 3 \
    -o "$CHECKPOINT.part" \
    https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
  mv "$CHECKPOINT.part" "$CHECKPOINT"
fi

echo "$CHECKPOINT_SHA256  $CHECKPOINT" | sha256sum --check --status || {
  echo "SAM checkpoint SHA-256 mismatch: $CHECKPOINT" >&2
  exit 1
}

PYTHONPATH="$SAM_DEPS:$SAM_REPO${PYTHONPATH:+:$PYTHONPATH}" \
  /usr/bin/python3 -c \
  'from sam2.build_sam import build_sam2; print("SAM 2.1 import: OK")'

echo "SAM 2.1 Hiera Tiny is ready for on-demand CPU loading."
