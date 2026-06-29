#!/usr/bin/env python3
"""Follow a red target using the Pi Camera and continuous RPLIDAR safety.

This is a small autonomy milestone, not a general navigation stack:

* camera: detect the horizontal direction of a red cup/target;
* lidar: continuously watch a forward sector and stop near obstacles;
* motors: use a TB6612 driver to arc left/right or drive forward.

Run only in a clear test area with one hand near the motor-battery switch.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

try:
    from gpiozero import DigitalOutputDevice, PWMOutputDevice
except ImportError as exc:  # pragma: no cover - only expected off the Pi.
    raise SystemExit(
        "gpiozero is required on the Raspberry Pi. Install with:\n"
        "  sudo apt install python3-gpiozero python3-lgpio"
    ) from exc


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
DEFAULT_LIDAR_STREAM = "/home/pi5/fuse-recorder/build/lidar_front_stream"
DEFAULT_OUTPUT_DIR = Path.home() / "sensor-tests"

PIN_STBY = 25

PIN_AIN1 = 23
PIN_AIN2 = 24
PIN_PWMA = 18

PIN_BIN1 = 5
PIN_BIN2 = 6
PIN_PWMB = 13


@dataclass(frozen=True)
class Target:
    cx: int
    cy: int
    error_x: float
    red_pixels: int
    source: str = "red_component"
    label: str = "red"
    score: float | None = None


@dataclass(frozen=True)
class ObjectBox:
    x0: int
    y0: int
    x1: int
    y1: int
    label: str
    class_id: int
    score: float

    @property
    def width(self) -> int:
        return self.x1 - self.x0 + 1

    @property
    def height(self) -> int:
        return self.y1 - self.y0 + 1


@dataclass(frozen=True)
class RedComponent:
    x0: int
    y0: int
    x1: int
    y1: int
    pixels: int
    cx: float
    cy: float

    @property
    def width(self) -> int:
        return self.x1 - self.x0 + 1

    @property
    def height(self) -> int:
        return self.y1 - self.y0 + 1

    @property
    def aspect(self) -> float:
        return self.width / max(1, self.height)


@dataclass(frozen=True)
class MotorCommand:
    left: float
    right: float


class Tb6612Drive:
    def __init__(
        self,
        *,
        left_trim: float,
        right_trim: float,
        reverse_left: bool = False,
        reverse_right: bool = False,
    ) -> None:
        self.left_trim = left_trim
        self.right_trim = right_trim
        self.reverse_left = reverse_left
        self.reverse_right = reverse_right
        self.stby = DigitalOutputDevice(PIN_STBY, initial_value=False)
        self.ain1 = DigitalOutputDevice(PIN_AIN1, initial_value=False)
        self.ain2 = DigitalOutputDevice(PIN_AIN2, initial_value=False)
        self.pwma = PWMOutputDevice(PIN_PWMA, initial_value=0.0)
        self.bin1 = DigitalOutputDevice(PIN_BIN1, initial_value=False)
        self.bin2 = DigitalOutputDevice(PIN_BIN2, initial_value=False)
        self.pwmb = PWMOutputDevice(PIN_PWMB, initial_value=0.0)

    def _set_one(
        self,
        *,
        speed: float,
        trim: float,
        reverse: bool,
        in1: DigitalOutputDevice,
        in2: DigitalOutputDevice,
        pwm: PWMOutputDevice,
    ) -> None:
        speed = max(-1.0, min(1.0, speed))
        if reverse:
            speed = -speed

        if speed > 0.0:
            in1.on()
            in2.off()
            pwm.value = min(1.0, speed * trim)
        elif speed < 0.0:
            in1.off()
            in2.on()
            pwm.value = min(1.0, -speed * trim)
        else:
            pwm.value = 0.0
            in1.off()
            in2.off()

    def drive(self, left: float, right: float) -> None:
        self.stby.on()
        self._set_one(
            speed=left,
            trim=self.left_trim,
            reverse=self.reverse_left,
            in1=self.ain1,
            in2=self.ain2,
            pwm=self.pwma,
        )
        # Stagger startup slightly; this helped the TT motors avoid stalls.
        time.sleep(0.08)
        self._set_one(
            speed=right,
            trim=self.right_trim,
            reverse=self.reverse_right,
            in1=self.bin1,
            in2=self.bin2,
            pwm=self.pwmb,
        )

    def stop(self) -> None:
        self.pwma.value = 0.0
        self.pwmb.value = 0.0
        self.ain1.off()
        self.ain2.off()
        self.bin1.off()
        self.bin2.off()
        self.stby.off()

    def close(self) -> None:
        self.stop()
        self.stby.close()
        self.ain1.close()
        self.ain2.close()
        self.pwma.close()
        self.bin1.close()
        self.bin2.close()
        self.pwmb.close()


class FrontDistanceState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.closest_m: float | None = None
        self.last_update_s = 0.0
        self.last_record: dict[str, Any] | None = None

    def update(self, record: dict[str, Any]) -> None:
        with self._lock:
            closest = record.get("closest_m")
            self.closest_m = float(closest) if closest is not None else None
            self.last_update_s = time.monotonic()
            self.last_record = record

    def snapshot(self) -> tuple[float | None, float, dict[str, Any] | None]:
        with self._lock:
            return self.closest_m, self.last_update_s, self.last_record


def bounded_unit(value: str) -> float:
    parsed = float(value)
    if not 0.0 <= parsed <= 1.0:
        raise argparse.ArgumentTypeError("value must be between 0.0 and 1.0")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def read_lidar_stream(
    process: subprocess.Popen[str],
    state: FrontDistanceState,
    shutdown: threading.Event,
) -> None:
    assert process.stdout is not None
    for line in process.stdout:
        if shutdown.is_set():
            break
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if record.get("type") == "front":
            state.update(record)


def red_components(mask: np.ndarray, *, min_pixels: int) -> list[RedComponent]:
    """Return 4-connected red components from a binary image mask.

    The runtime image is intentionally small, usually 640x360. A stack-based
    component walk is fast enough, avoids adding OpenCV as a runtime dependency,
    and lets us reject isolated red clutter instead of averaging every red pixel.
    """
    height, width = mask.shape
    visited = np.zeros(mask.shape, dtype=np.bool_)
    components: list[RedComponent] = []
    true_ys, true_xs = np.where(mask)

    for start_y, start_x in zip(true_ys.tolist(), true_xs.tolist(), strict=True):
        if visited[start_y, start_x]:
            continue

        stack = [(start_y, start_x)]
        visited[start_y, start_x] = True
        count = 0
        sum_x = 0
        sum_y = 0
        x0 = x1 = start_x
        y0 = y1 = start_y

        while stack:
            y, x = stack.pop()
            count += 1
            sum_x += x
            sum_y += y
            x0 = min(x0, x)
            x1 = max(x1, x)
            y0 = min(y0, y)
            y1 = max(y1, y)

            for ny, nx in (
                (y - 1, x),
                (y + 1, x),
                (y, x - 1),
                (y, x + 1),
            ):
                if (
                    0 <= ny < height
                    and 0 <= nx < width
                    and not visited[ny, nx]
                    and mask[ny, nx]
                ):
                    visited[ny, nx] = True
                    stack.append((ny, nx))

        if count >= min_pixels:
            components.append(
                RedComponent(
                    x0=x0,
                    y0=y0,
                    x1=x1,
                    y1=y1,
                    pixels=count,
                    cx=sum_x / count,
                    cy=sum_y / count,
                )
            )

    return components


def component_reason(
    component: RedComponent,
    *,
    width: int,
    height: int,
    args: argparse.Namespace,
) -> str | None:
    if component.pixels < getattr(args, "min_red_pixels", 150):
        return "too_few_pixels"
    if component.width < getattr(args, "target_min_width_px", 8):
        return "too_narrow"
    if component.height < getattr(args, "target_min_height_px", 10):
        return "too_short"
    if component.width > width * getattr(args, "target_max_width_frac", 0.70):
        return "too_wide"
    if component.height > height * getattr(args, "target_max_height_frac", 0.90):
        return "too_tall"
    if component.cy < height * getattr(args, "target_min_center_y_frac", 0.20):
        return "too_high_in_image"
    if component.aspect < getattr(args, "target_min_aspect", 0.20):
        return "aspect_too_skinny"
    if component.aspect > getattr(args, "target_max_aspect", 3.50):
        return "aspect_too_wide"
    return None


def component_score(component: RedComponent, *, width: int) -> float:
    image_center_x = width / 2.0
    center_penalty = abs(component.cx - image_center_x) / max(1.0, image_center_x)
    return component.pixels * (1.0 - 0.20 * center_penalty)


def choose_red_component(
    components: list[RedComponent],
    *,
    width: int,
    height: int,
    args: argparse.Namespace,
) -> tuple[RedComponent | None, list[tuple[RedComponent, str | None]]]:
    judged = [
        (
            component,
            component_reason(component, width=width, height=height, args=args),
        )
        for component in components
    ]
    candidates = [component for component, reason in judged if reason is None]
    if not candidates:
        return None, judged
    return max(candidates, key=lambda component: component_score(component, width=width)), judged


def component_to_dict(component: RedComponent, reason: str | None) -> dict[str, Any]:
    return {
        "x0": component.x0,
        "y0": component.y0,
        "x1": component.x1,
        "y1": component.y1,
        "width": component.width,
        "height": component.height,
        "pixels": component.pixels,
        "cx": component.cx,
        "cy": component.cy,
        "aspect": component.aspect,
        "reason": reason,
    }


def parse_csv_text(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def parse_csv_ints(value: str) -> set[int]:
    result: set[int] = set()
    for item in parse_csv_text(value):
        try:
            result.add(int(item))
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"expected comma-separated integers, got {item!r}"
            ) from exc
    return result


def load_detector_labels(path: Path) -> list[str]:
    labels: list[str] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split(maxsplit=1)
        if len(parts) == 2 and parts[0].isdigit():
            labels.append(parts[1].strip())
        else:
            labels.append(line)
    if labels and labels[0] in {"???", "__background__", "background"}:
        # Common TFLite COCO label maps include a placeholder first label, while
        # object-detection class outputs are 0-based for real labels.
        labels = labels[1:]
    return labels


class TFLiteObjectDetector:
    def __init__(self, args: argparse.Namespace) -> None:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            try:
                from tensorflow.lite.python.interpreter import Interpreter  # type: ignore
            except ImportError as exc:
                raise SystemExit(
                    "Object detection was requested with --detector-model, but "
                    "neither tflite_runtime nor tensorflow is installed on the Pi.\n"
                    "Try:\n"
                    "  sudo apt install python3-tflite-runtime\n"
                    "or install a TensorFlow Lite runtime for your Raspberry Pi OS."
                ) from exc

        if args.detector_labels is None:
            raise SystemExit(
                "--detector-labels is required with --detector-model so the "
                "robot can identify the COCO 'cup' class."
            )

        self.labels = load_detector_labels(args.detector_labels)
        self.target_labels = {
            label.lower() for label in parse_csv_text(args.detector_target_labels)
        }
        self.target_class_ids = parse_csv_ints(args.detector_target_class_ids)
        self.min_score = args.detector_confidence
        self.label_offset = args.detector_label_offset
        self.interpreter = Interpreter(
            model_path=str(args.detector_model),
            num_threads=args.detector_threads,
        )
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        input_shape = self.input_details[0]["shape"]
        self.input_height = int(input_shape[1])
        self.input_width = int(input_shape[2])
        self.input_dtype = self.input_details[0]["dtype"]

    def label_for_class(self, class_id: int) -> str:
        index = class_id + self.label_offset
        if 0 <= index < len(self.labels):
            return self.labels[index]
        return f"class_{class_id}"

    def is_target(self, detection: ObjectBox) -> bool:
        if detection.class_id in self.target_class_ids:
            return True
        return detection.label.lower() in self.target_labels

    def detect(self, image: Image.Image) -> list[ObjectBox]:
        resized = image.resize((self.input_width, self.input_height))
        input_data = np.expand_dims(np.array(resized), axis=0)
        if self.input_dtype == np.float32:
            input_data = (input_data.astype(np.float32) - 127.5) / 127.5
        else:
            input_data = input_data.astype(self.input_dtype)

        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        self.interpreter.invoke()

        outputs = [
            self.interpreter.get_tensor(detail["index"])
            for detail in self.output_details
        ]
        boxes_arr: np.ndarray | None = None
        classes_arr: np.ndarray | None = None
        scores_arr: np.ndarray | None = None

        for output in outputs:
            arr = np.squeeze(output)
            if arr.ndim == 2 and arr.shape[-1] == 4:
                boxes_arr = arr
            elif arr.ndim == 1:
                if np.issubdtype(arr.dtype, np.floating) and arr.size > 1 and float(np.max(arr)) <= 1.0:
                    scores_arr = arr
                elif arr.size > 1:
                    classes_arr = arr

        if boxes_arr is None or classes_arr is None or scores_arr is None:
            raise RuntimeError(
                "Could not interpret detector outputs. Expected standard "
                "TFLite object-detection tensors: boxes, classes, scores."
            )

        width, height = image.size
        detections: list[ObjectBox] = []
        count = min(len(boxes_arr), len(classes_arr), len(scores_arr))
        for index in range(count):
            score = float(scores_arr[index])
            if score < self.min_score:
                continue
            ymin, xmin, ymax, xmax = [float(value) for value in boxes_arr[index]]
            if xmax <= 1.5 and ymax <= 1.5:
                x0 = int(round(xmin * width))
                y0 = int(round(ymin * height))
                x1 = int(round(xmax * width))
                y1 = int(round(ymax * height))
            else:
                x0 = int(round(xmin))
                y0 = int(round(ymin))
                x1 = int(round(xmax))
                y1 = int(round(ymax))
            x0 = max(0, min(width - 1, x0))
            y0 = max(0, min(height - 1, y0))
            x1 = max(0, min(width - 1, x1))
            y1 = max(0, min(height - 1, y1))
            if x1 <= x0 or y1 <= y0:
                continue
            class_id = int(classes_arr[index])
            detections.append(
                ObjectBox(
                    x0=x0,
                    y0=y0,
                    x1=x1,
                    y1=y1,
                    class_id=class_id,
                    label=self.label_for_class(class_id),
                    score=score,
                )
            )
        return detections


def get_object_detector(args: argparse.Namespace) -> TFLiteObjectDetector | None:
    model = getattr(args, "detector_model", None)
    if model is None:
        return None
    cached = getattr(args, "_object_detector", None)
    if cached is None:
        cached = TFLiteObjectDetector(args)
        setattr(args, "_object_detector", cached)
    return cached


def box_to_dict(box: ObjectBox, *, red_pixels: int, red_fraction: float, reason: str | None) -> dict[str, Any]:
    return {
        "x0": box.x0,
        "y0": box.y0,
        "x1": box.x1,
        "y1": box.y1,
        "width": box.width,
        "height": box.height,
        "label": box.label,
        "class_id": box.class_id,
        "score": box.score,
        "red_pixels": red_pixels,
        "red_fraction": red_fraction,
        "reason": reason,
    }


def choose_detector_target(
    detections: list[ObjectBox],
    *,
    mask: np.ndarray,
    args: argparse.Namespace,
) -> tuple[ObjectBox | None, int, float, list[dict[str, Any]]]:
    judged: list[dict[str, Any]] = []
    candidates: list[tuple[ObjectBox, int, float]] = []

    for box in detections:
        crop = mask[box.y0 : box.y1 + 1, box.x0 : box.x1 + 1]
        red_pixels = int(np.count_nonzero(crop))
        area = max(1, box.width * box.height)
        red_fraction = red_pixels / area
        detector: TFLiteObjectDetector = getattr(args, "_object_detector")

        reason: str | None = None
        if not detector.is_target(box):
            reason = "not_target_label"
        elif red_pixels < args.min_red_pixels:
            reason = "too_few_red_pixels"
        elif red_fraction < args.detector_min_red_fraction:
            reason = "too_little_red_in_cup_box"

        judged.append(
            box_to_dict(
                box,
                red_pixels=red_pixels,
                red_fraction=red_fraction,
                reason=reason,
            )
        )
        if reason is None:
            candidates.append((box, red_pixels, red_fraction))

    if not candidates:
        return None, 0, 0.0, judged

    def score(item: tuple[ObjectBox, int, float]) -> float:
        box, red_pixels, red_fraction = item
        return box.score * (1.0 + red_fraction) * max(1, red_pixels)

    selected, red_pixels, red_fraction = max(candidates, key=score)
    return selected, red_pixels, red_fraction, judged


def detect_red_target(args: argparse.Namespace) -> Target | None:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    image_path = args.output_dir / "red-cup-continuous.jpg"
    debug_path = args.output_dir / "red-cup-continuous-detection.jpg"
    debug_json_path = args.output_dir / "red-cup-continuous-detection.json"

    subprocess.run(
        [
            "rpicam-still",
            "--nopreview",
            "--immediate",
            "--timeout",
            str(args.camera_timeout_ms),
            "--width",
            str(args.image_width),
            "--height",
            str(args.image_height),
            "-o",
            str(image_path),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    image = Image.open(image_path).convert("RGB")
    arr = np.array(image).astype(np.int16)
    red = arr[:, :, 0]
    green = arr[:, :, 1]
    blue = arr[:, :, 2]

    mask = (
        (red > args.red_min)
        & (red > green * args.red_green_ratio)
        & (red > blue * args.red_blue_ratio)
    )
    ys, xs = np.where(mask)

    draw = ImageDraw.Draw(image)
    width, height = image.size
    draw.line([(width // 2, 0), (width // 2, height)], fill=(0, 0, 255), width=3)
    target_min_y = int(height * getattr(args, "target_min_center_y_frac", 0.20))
    draw.line([(0, target_min_y), (width, target_min_y)], fill=(255, 255, 0), width=2)

    detector = get_object_detector(args)
    if detector is not None:
        detections = detector.detect(image)
        selected_box, red_pixels, red_fraction, judged_boxes = choose_detector_target(
            detections,
            mask=mask,
            args=args,
        )
        for item in judged_boxes:
            outline = (255, 255, 0) if item["reason"] is None else (255, 140, 0)
            draw.rectangle(
                [(item["x0"], item["y0"]), (item["x1"], item["y1"])],
                outline=outline,
                width=2,
            )
        if selected_box is not None:
            cx = int(round((selected_box.x0 + selected_box.x1) / 2.0))
            cy = int(round((selected_box.y0 + selected_box.y1) / 2.0))
            error_x = cx - width / 2
            draw.rectangle(
                [(selected_box.x0, selected_box.y0), (selected_box.x1, selected_box.y1)],
                outline=(0, 255, 0),
                width=4,
            )
            draw.line([(cx, 0), (cx, height)], fill=(0, 255, 0), width=3)
            draw.ellipse(
                [(cx - 8, cy - 8), (cx + 8, cy + 8)],
                outline=(0, 255, 0),
                width=4,
            )
            image.save(debug_path)
            debug_json_path.write_text(
                json.dumps(
                    {
                        "mode": "object_detector",
                        "selected": box_to_dict(
                            selected_box,
                            red_pixels=red_pixels,
                            red_fraction=red_fraction,
                            reason=None,
                        ),
                        "total_red_pixels": int(len(xs)),
                        "detections": judged_boxes,
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
            return Target(
                cx=cx,
                cy=cy,
                error_x=float(error_x),
                red_pixels=red_pixels,
                source="detector",
                label=selected_box.label,
                score=selected_box.score,
            )

        image.save(debug_path)
        debug_json_path.write_text(
            json.dumps(
                {
                    "mode": "object_detector",
                    "selected": None,
                    "total_red_pixels": int(len(xs)),
                    "detections": judged_boxes,
                    "reason": "no_detected_cup_box_with_enough_red",
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        if not getattr(args, "fallback_to_red_blob", False):
            return None

    if len(xs) < args.min_red_pixels:
        image.save(debug_path)
        debug_json_path.write_text(
            json.dumps(
                {
                    "mode": "red_component",
                    "selected": None,
                    "total_red_pixels": int(len(xs)),
                    "components": [],
                    "reason": "too_few_total_red_pixels",
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return None

    components = red_components(
        mask,
        min_pixels=max(
            1,
            min(args.min_red_pixels, getattr(args, "component_seed_min_pixels", 20)),
        ),
    )
    selected, judged = choose_red_component(
        components,
        width=width,
        height=height,
        args=args,
    )

    for component, reason in judged:
        outline = (255, 255, 0) if reason is None else (255, 140, 0)
        draw.rectangle(
            [(component.x0, component.y0), (component.x1, component.y1)],
            outline=outline,
            width=2,
        )

    if selected is None:
        image.save(debug_path)
        debug_json_path.write_text(
            json.dumps(
                {
                    "mode": "red_component",
                    "selected": None,
                    "total_red_pixels": int(len(xs)),
                    "components": [
                        component_to_dict(component, reason)
                        for component, reason in judged
                    ],
                    "reason": "no_component_passed_filters",
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return None

    x0, x1 = selected.x0, selected.x1
    y0, y1 = selected.y0, selected.y1
    cx = int(round(selected.cx))
    cy = int(round(selected.cy))
    error_x = cx - width / 2

    draw.rectangle([(x0, y0), (x1, y1)], outline=(0, 255, 0), width=4)
    draw.line([(cx, 0), (cx, height)], fill=(0, 255, 0), width=3)
    draw.ellipse([(cx - 8, cy - 8), (cx + 8, cy + 8)], outline=(0, 255, 0), width=4)
    image.save(debug_path)
    debug_json_path.write_text(
        json.dumps(
            {
                "mode": "red_component",
                "selected": component_to_dict(selected, None),
                "total_red_pixels": int(len(xs)),
                "components": [
                    component_to_dict(component, reason)
                    for component, reason in judged
                ],
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    return Target(
        cx=cx,
        cy=cy,
        error_x=float(error_x),
        red_pixels=selected.pixels,
        source="red_component",
        label="red_blob",
        score=None,
    )


def start_lidar_stream(args: argparse.Namespace) -> tuple[subprocess.Popen[str], Any]:
    lidar_log_path = args.output_dir / "red-cup-lidar-stream.log"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    lidar_log = lidar_log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            str(args.lidar_stream),
            args.port,
            str(args.baudrate),
            str(args.front_center_deg),
            str(args.front_half_width_deg),
        ],
        stdout=subprocess.PIPE,
        stderr=lidar_log,
        text=True,
        bufsize=1,
    )
    return process, lidar_log


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Continuously follow a red cup with persistent lidar safety."
    )
    parser.add_argument("--armed", action="store_true", help="Required safety acknowledgement.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--lidar-stream", type=Path, default=Path(DEFAULT_LIDAR_STREAM))
    parser.add_argument("--front-center-deg", type=float, default=85.0)
    parser.add_argument("--front-half-width-deg", type=float, default=20.0)
    parser.add_argument("--stop-distance-m", type=positive_float, default=0.203)
    parser.add_argument("--max-run-s", type=positive_float, default=45.0)
    parser.add_argument("--stale-lidar-s", type=positive_float, default=1.0)
    parser.add_argument("--left-trim", type=bounded_unit, default=0.95)
    parser.add_argument("--right-trim", type=bounded_unit, default=0.85)
    parser.add_argument("--forward-speed", type=bounded_unit, default=0.48)
    parser.add_argument("--arc-slow", type=bounded_unit, default=0.52)
    parser.add_argument("--arc-fast", type=bounded_unit, default=0.60)
    parser.add_argument("--center-deadband-px", type=int, default=100)
    parser.add_argument("--min-red-pixels", type=int, default=150)
    parser.add_argument("--component-seed-min-pixels", type=int, default=20)
    parser.add_argument("--target-min-width-px", type=int, default=8)
    parser.add_argument("--target-min-height-px", type=int, default=10)
    parser.add_argument("--target-min-center-y-frac", type=float, default=0.20)
    parser.add_argument("--target-max-width-frac", type=float, default=0.70)
    parser.add_argument("--target-max-height-frac", type=float, default=0.90)
    parser.add_argument("--target-min-aspect", type=float, default=0.20)
    parser.add_argument("--target-max-aspect", type=float, default=3.50)
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=360)
    parser.add_argument("--camera-timeout-ms", type=int, default=300)
    parser.add_argument("--red-min", type=int, default=100)
    parser.add_argument("--red-green-ratio", type=float, default=1.45)
    parser.add_argument("--red-blue-ratio", type=float, default=1.45)
    parser.add_argument("--detector-model", type=Path, default=None)
    parser.add_argument("--detector-labels", type=Path, default=None)
    parser.add_argument("--detector-target-labels", default="cup")
    parser.add_argument("--detector-target-class-ids", default="")
    parser.add_argument("--detector-confidence", type=float, default=0.35)
    parser.add_argument("--detector-label-offset", type=int, default=0)
    parser.add_argument("--detector-threads", type=int, default=2)
    parser.add_argument("--detector-min-red-fraction", type=float, default=0.03)
    parser.add_argument("--fallback-to-red-blob", action="store_true")
    parser.add_argument("--max-lost-steps", type=int, default=12)
    parser.add_argument("--reverse-left", action="store_true")
    parser.add_argument("--reverse-right", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    if not args.armed:
        parser.error("--armed is required; verify a clear area and keep hand near switch")
    if not args.lidar_stream.exists():
        parser.error(f"lidar stream helper does not exist: {args.lidar_stream}")
    if not Path(args.port).exists():
        parser.error(f"lidar serial port does not exist: {args.port}")

    drive = Tb6612Drive(
        left_trim=args.left_trim,
        right_trim=args.right_trim,
        reverse_left=args.reverse_left,
        reverse_right=args.reverse_right,
    )
    front_state = FrontDistanceState()
    shutdown = threading.Event()
    emergency_stop = threading.Event()
    lidar_process: subprocess.Popen[str] | None = None
    lidar_log = None

    def stop_for_safety(message: str) -> None:
        print(message, flush=True)
        emergency_stop.set()
        drive.stop()

    def safety_watchdog() -> None:
        while not shutdown.is_set() and not emergency_stop.is_set():
            closest_m, last_update_s, _ = front_state.snapshot()
            now = time.monotonic()
            if last_update_s > 0.0 and now - last_update_s > args.stale_lidar_s:
                stop_for_safety(
                    f"SAFETY STOP: lidar stream stale for {now - last_update_s:.2f}s"
                )
                return
            if closest_m is not None and closest_m <= args.stop_distance_m:
                stop_for_safety(
                    "SAFETY STOP: "
                    f"front={closest_m:.3f} m <= {args.stop_distance_m:.3f} m"
                )
                return
            time.sleep(0.03)

    try:
        lidar_process, lidar_log = start_lidar_stream(args)
        reader_thread = threading.Thread(
            target=read_lidar_stream,
            args=(lidar_process, front_state, shutdown),
            daemon=True,
        )
        watchdog_thread = threading.Thread(target=safety_watchdog, daemon=True)
        reader_thread.start()

        print("Waiting for lidar stream...")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            _, last_update_s, _ = front_state.snapshot()
            if last_update_s > 0.0:
                break
            time.sleep(0.05)

        closest_m, last_update_s, _ = front_state.snapshot()
        if last_update_s == 0.0:
            print("No lidar stream received. Not moving.", file=sys.stderr)
            return 3

        watchdog_thread.start()

        print(
            "Continuous red cup follow. "
            f"Stop distance={args.stop_distance_m:.3f} m. Hand ready."
        )
        print(f"Initial front distance={closest_m}")
        time.sleep(2.0)

        start_s = time.monotonic()
        step = 0
        last_seen_direction = "center"
        lost_count = 0

        while (
            time.monotonic() - start_s < args.max_run_s
            and not emergency_stop.is_set()
        ):
            step += 1
            closest_m, _, _ = front_state.snapshot()
            if closest_m is None:
                print(f"{step}: no front lidar distance -> stop")
                drive.stop()
                time.sleep(0.1)
                continue

            target = detect_red_target(args)
            if emergency_stop.is_set():
                break

            if target is None:
                lost_count += 1
                print(
                    f"{step}: no red cup; front={closest_m:.3f} "
                    f"-> search {last_seen_direction}"
                )
                if lost_count > args.max_lost_steps:
                    print("lost too long -> stop")
                    drive.stop()
                    continue
                if last_seen_direction == "right":
                    drive.drive(args.arc_fast, args.arc_slow)
                else:
                    drive.drive(args.arc_slow, args.arc_fast)
                continue

            lost_count = 0
            print(
                f"{step}: front={closest_m:.3f} "
                f"cup error_x={target.error_x:.1f} "
                f"red_pixels={target.red_pixels} "
                f"source={target.source} label={target.label}"
            )

            if target.error_x < -args.center_deadband_px:
                last_seen_direction = "left"
                drive.drive(args.arc_slow, args.arc_fast)
            elif target.error_x > args.center_deadband_px:
                last_seen_direction = "right"
                drive.drive(args.arc_fast, args.arc_slow)
            else:
                last_seen_direction = "center"
                drive.drive(args.forward_speed, args.forward_speed)

        if emergency_stop.is_set():
            print("STOP_CONDITION_REACHED")
        else:
            print("MAX_RUN_SECONDS reached")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; stopping", file=sys.stderr)
        return 130
    finally:
        print("Stopping")
        shutdown.set()
        drive.stop()
        drive.close()
        if lidar_process is not None and lidar_process.poll() is None:
            lidar_process.send_signal(signal.SIGINT)
            try:
                lidar_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                lidar_process.kill()
        if lidar_log is not None:
            lidar_log.close()
        print(f"Debug image: {args.output_dir / 'red-cup-continuous-detection.jpg'}")


if __name__ == "__main__":
    raise SystemExit(main())
