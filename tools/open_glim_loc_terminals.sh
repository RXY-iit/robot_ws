#!/usr/bin/env bash
# Compatibility wrapper. The GLIM localization launcher was replaced by the
# GICP localization launcher.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[open_glim_loc_terminals] renamed to open_gicp_loc_terminals.sh"
exec "$SCRIPT_DIR/open_gicp_loc_terminals.sh" "$@"
