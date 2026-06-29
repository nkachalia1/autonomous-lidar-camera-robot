#!/usr/bin/env bash
set -euo pipefail

# Install a lightweight COCO SSD MobileNet TFLite model for the Pi Camera v2
# red-cup robot behavior.
#
# This does not commit model binaries to Git. It places them under:
#   ~/models/coco_ssd_mobilenet_v1

MODEL_DIR="${HOME}/models/coco_ssd_mobilenet_v1"
MODEL_ZIP="${MODEL_DIR}/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip"
MODEL_URL="https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip"

mkdir -p "${MODEL_DIR}"

if ! python3 - <<'PY'
try:
    from tflite_runtime.interpreter import Interpreter
    print("tflite_runtime: ok")
except ImportError:
    raise SystemExit(1)
PY
then
  echo "Installing TensorFlow Lite runtime from apt..."
  sudo apt update
  sudo apt install -y python3-tflite-runtime
fi

if ! command -v unzip >/dev/null 2>&1; then
  sudo apt update
  sudo apt install -y unzip
fi

if ! command -v curl >/dev/null 2>&1; then
  sudo apt update
  sudo apt install -y curl
fi

if [ ! -f "${MODEL_DIR}/detect.tflite" ] || [ ! -f "${MODEL_DIR}/labelmap.txt" ]; then
  echo "Downloading COCO SSD MobileNet TFLite model..."
  curl -L "${MODEL_URL}" -o "${MODEL_ZIP}"
  unzip -o "${MODEL_ZIP}" -d "${MODEL_DIR}"
fi

echo "Model ready:"
echo "  ${MODEL_DIR}/detect.tflite"
echo "  ${MODEL_DIR}/labelmap.txt"
echo
echo "Use with:"
echo "  python3 ~/red_cup_search_and_approach.py --armed \\"
echo "    --detector-model ${MODEL_DIR}/detect.tflite \\"
echo "    --detector-labels ${MODEL_DIR}/labelmap.txt"
