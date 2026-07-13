#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${1:-/home/ruan-x/robot_ws/omniverse_asset_packs}"
ZIP_DIR="${ROOT_DIR}/zips"
EXTRACT_DIR="${ROOT_DIR}/extracted"

mkdir -p "${ZIP_DIR}" "${EXTRACT_DIR}"

download_one() {
  local name="$1"
  local url="$2"
  local out="${ZIP_DIR}/${name}"

  echo "==> Downloading ${name}"
  echo "    ${url}"
  curl -L \
    --fail \
    --continue-at - \
    --retry 8 \
    --retry-delay 10 \
    --connect-timeout 30 \
    --output "${out}" \
    "${url}"
}

download_one "Commercial_NVD@10013.zip" \
  "https://d4i3qtqj3r0z5.cloudfront.net/Commercial_NVD%4010013.zip"

download_one "SimReady_Warehouse_01_NVD@10010.zip" \
  "https://d4i3qtqj3r0z5.cloudfront.net/SimReady_Warehouse_01_NVD%4010010.zip"

download_one "SimReady_Furniture_Misc_01_NVD@10010.zip" \
  "https://d4i3qtqj3r0z5.cloudfront.net/SimReady_Furniture_Misc_01_NVD%4010010.zip"

download_one "Base_Materials_NVD@10013.zip" \
  "https://d4i3qtqj3r0z5.cloudfront.net/Base_Materials_NVD%4010013.zip"

echo
echo "Downloads complete:"
echo "  ${ZIP_DIR}"
echo
echo "Unzip manually, or run:"
echo "  unzip -n '${ZIP_DIR}/Commercial_NVD@10013.zip' -d '${EXTRACT_DIR}/Commercial_NVD@10013'"
echo "  unzip -n '${ZIP_DIR}/SimReady_Warehouse_01_NVD@10010.zip' -d '${EXTRACT_DIR}/SimReady_Warehouse_01_NVD@10010'"
echo "  unzip -n '${ZIP_DIR}/SimReady_Furniture_Misc_01_NVD@10010.zip' -d '${EXTRACT_DIR}/SimReady_Furniture_Misc_01_NVD@10010'"
echo "  unzip -n '${ZIP_DIR}/Base_Materials_NVD@10013.zip' -d '${EXTRACT_DIR}/Base_Materials_NVD@10013'"
