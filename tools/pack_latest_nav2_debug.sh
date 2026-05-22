#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
ROOT="${DEBUG_OUTPUT_ROOT:-$WS/debug-output}"

latest="$(find "$ROOT" -maxdepth 1 -type d -name 'nav2_*' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-)"
if [[ -z "${latest:-}" ]]; then
  echo "No nav2 debug session found under $ROOT" >&2
  exit 1
fi

archive="$ROOT/$(basename "$latest").tar.gz"
tar -C "$ROOT" -czf "$archive" "$(basename "$latest")"
echo "$archive"
