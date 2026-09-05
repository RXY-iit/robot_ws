#!/usr/bin/env bash
# Builds the dashboard's React/TypeScript frontend (src/robot_agent/web) and
# leaves the output in web/dist/. `colcon build` does not do this automatically
# (robot_agent is an ament_python package, no build hook) -- run this first,
# then `colcon build --packages-select robot_agent` to install the built assets
# into share/robot_agent/web/.
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../src/robot_agent/web"
npm install
npm run build
