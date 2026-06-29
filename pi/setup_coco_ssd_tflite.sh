#!/usr/bin/env bash
set -euo pipefail

# Install a lightweight COCO SSD MobileNet TFLite model for the Pi Camera v2
# red-cup robot behavior.
#
# This does not commit model binaries to Git. It places them under:
#   ~/models/coco_ssd_mobilenet_v1
#
# Runtime install notes:
# - Some Raspberry Pi OS images do not provide python3-tflite-runtime in apt.
# - Bookworm also blocks system-wide pip installs through PEP 668.
# - We therefore prefer a small venv at ~/fuse-venv with system site packages
#   visible, so gpiozero/numpy/Pillow packages installed by apt remain usable.

MODEL_DIR="${HOME}/models/coco_ssd_mobilenet_v1"
MODEL_ZIP="${MODEL_DIR}/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip"
MODEL_URL="https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip"
VENV_DIR="${HOME}/fuse-venv"

mkdir -p "${MODEL_DIR}"

if python3 - <<'PY'
try:
    from tflite_runtime.interpreter import Interpreter
    print("system tflite_runtime: ok")
except ImportError:
    raise SystemExit(1)
PY
then
  PYTHON_CMD="python3"
else
  echo "System tflite_runtime is not available."
  echo "Trying apt package first, then falling back to a local venv..."
  sudo apt update
  if apt-cache show python3-tflite-runtime >/dev/null 2>&1; then
    sudo apt install -y python3-tflite-runtime
    PYTHON_CMD="python3"
  else
    sudo apt install -y python3-venv python3-pip python3-numpy python3-pil
    if [ ! -d "${VENV_DIR}" ]; then
      python3 -m venv --system-site-packages "${VENV_DIR}"
    fi
    "${VENV_DIR}/bin/python" -m pip install --upgrade pip
    "${VENV_DIR}/bin/python" -m pip install \
      --extra-index-url https://www.piwheels.org/simple \
      tflite-runtime
    PYTHON_CMD="${VENV_DIR}/bin/python"
  fi
fi

"${PYTHON_CMD}" - <<'PY'
from tflite_runtime.interpreter import Interpreter
print("tflite_runtime import: ok")
PY

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
echo "  ${PYTHON_CMD} ~/red_cup_search_and_approach.py --armed \\"
echo "    --detector-model ${MODEL_DIR}/detect.tflite \\"
echo "    --detector-labels ${MODEL_DIR}/labelmap.txt"
