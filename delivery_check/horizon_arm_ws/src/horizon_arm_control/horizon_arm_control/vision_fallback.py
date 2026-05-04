from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import cv2
import numpy as np


COCO_CLASS_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

_YOLO_NET_CACHE: dict[str, Any] = {}


def parse_options_json(value: str) -> dict[str, Any]:
    text = str(value or "").strip()
    if not text:
        return {}
    payload = json.loads(text)
    if not isinstance(payload, dict):
        raise ValueError("options_json must be a JSON object")
    return payload


def capture_frame(camera_id: int):
    cap = cv2.VideoCapture(int(camera_id))
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"failed to open camera {camera_id}")
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError(f"failed to capture frame from camera {camera_id}")
    return frame


def resolve_model_path(
    *,
    explicit_path: str = "",
    sdk_root: str = "",
    config: dict[str, Any] | None = None,
) -> str:
    explicit = str(explicit_path or "").strip()
    if explicit and Path(explicit).exists():
        return explicit

    config = config or {}
    config_path = str(config.get("model_path", "")).strip()
    if config_path and Path(config_path).exists():
        return config_path

    env_cfg_dir = os.environ.get("HORIZONARM_CONFIG_DIR", "").strip()
    if env_cfg_dir:
        candidate = Path(env_cfg_dir) / "yolov8n.onnx"
        if candidate.exists():
            return str(candidate)

    env_data_root = os.environ.get("HORIZON_DATA_DIR", "").strip()
    if env_data_root:
        candidate = Path(env_data_root) / "config" / "yolov8n.onnx"
        if candidate.exists():
            return str(candidate)

    sdk_root_text = str(sdk_root or "").strip()
    if sdk_root_text:
        candidate = Path(sdk_root_text) / "config" / "yolov8n.onnx"
        if candidate.exists():
            return str(candidate)

    local_candidate = Path("config") / "yolov8n.onnx"
    if local_candidate.exists():
        return str(local_candidate)

    raise FileNotFoundError("unable to locate yolov8n.onnx")


def sample_hsv(frame, u: float, v: float, window_size: int) -> dict[str, Any]:
    if frame is None:
        raise ValueError("frame is empty")
    height, width = frame.shape[:2]
    cx = int(round(float(u)))
    cy = int(round(float(v)))
    half = max(1, int(window_size) // 2)
    x1 = max(0, cx - half)
    y1 = max(0, cy - half)
    x2 = min(width, cx + half + 1)
    y2 = min(height, cy + half + 1)
    if x1 >= x2 or y1 >= y2:
        raise ValueError("sampling window is outside the frame")

    roi = frame[y1:y2, x1:x2]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    center_bgr = frame[min(max(cy, 0), height - 1), min(max(cx, 0), width - 1)]
    center_hsv = cv2.cvtColor(
        np.uint8([[center_bgr]]),
        cv2.COLOR_BGR2HSV,
    )[0, 0]

    return {
        "success": True,
        "h": int(center_hsv[0]),
        "s": int(center_hsv[1]),
        "v": int(center_hsv[2]),
        "h_min": int(np.min(hsv_roi[:, :, 0])),
        "h_max": int(np.max(hsv_roi[:, :, 0])),
        "s_min": int(np.min(hsv_roi[:, :, 1])),
        "s_max": int(np.max(hsv_roi[:, :, 1])),
        "v_min": int(np.min(hsv_roi[:, :, 2])),
        "v_max": int(np.max(hsv_roi[:, :, 2])),
        "depth_m": 0.0,
        "message": "HSV sampled from RGB frame; depth is unavailable in fallback mode",
    }


def detect_hsv_targets(
    frame,
    *,
    hsv_range: list[int],
    target_class: str = "",
    min_area: float = 80.0,
) -> list[dict[str, Any]]:
    if frame is None:
        return []
    if len(hsv_range) != 6:
        raise ValueError("hsv_range must be [h_min,h_max,s_min,s_max,v_min,v_max]")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([int(hsv_range[0]), int(hsv_range[2]), int(hsv_range[4])], dtype=np.uint8)
    upper = np.array([int(hsv_range[1]), int(hsv_range[3]), int(hsv_range[5])], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: list[dict[str, Any]] = []
    frame_area = float(frame.shape[0] * frame.shape[1])
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < float(min_area):
            continue
        x, y, w, h = cv2.boundingRect(contour)
        center = [float(x + w / 2.0), float(y + h / 2.0)]
        score = min(1.0, area / max(1.0, frame_area))
        detections.append(
            {
                "bbox": [float(x), float(y), float(x + w), float(y + h)],
                "center": center,
                "score": score,
                "class_name": str(target_class or "hsv_target"),
                "depth_m": 0.0,
            }
        )
    detections.sort(key=lambda item: item.get("score", 0.0), reverse=True)
    return detections


def detect_yolo_targets(
    frame,
    *,
    model_path: str,
    target_class: str = "",
    conf_thres: float = 0.5,
    iou_thres: float = 0.45,
    class_names: list[str] | None = None,
) -> list[dict[str, Any]]:
    if frame is None:
        return []
    model_path = str(model_path)
    if model_path not in _YOLO_NET_CACHE:
        _YOLO_NET_CACHE[model_path] = cv2.dnn.readNetFromONNX(model_path)
    net = _YOLO_NET_CACHE[model_path]

    image_h, image_w = frame.shape[:2]
    input_size = 640
    blob = cv2.dnn.blobFromImage(
        frame,
        scalefactor=1.0 / 255.0,
        size=(input_size, input_size),
        mean=(0.0, 0.0, 0.0),
        swapRB=True,
        crop=False,
    )
    net.setInput(blob)
    outputs = net.forward()
    predictions = _normalize_yolo_output(outputs)
    names = list(class_names or COCO_CLASS_NAMES)

    boxes: list[list[float]] = []
    scores: list[float] = []
    class_ids: list[int] = []
    x_factor = float(image_w) / float(input_size)
    y_factor = float(image_h) / float(input_size)

    for row in predictions:
        row = np.asarray(row, dtype=np.float32).flatten()
        if row.size < 6:
            continue

        if row.size == len(names) + 5:
            objectness = float(row[4])
            class_scores = row[5:]
            class_id = int(np.argmax(class_scores))
            score = objectness * float(class_scores[class_id])
        else:
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])

        if score < float(conf_thres):
            continue

        class_name = names[class_id] if 0 <= class_id < len(names) else str(class_id)
        wanted = str(target_class or "").strip().lower()
        if wanted and class_name.lower() != wanted:
            continue

        cx, cy, w, h = [float(v) for v in row[:4]]
        x1 = (cx - w / 2.0) * x_factor
        y1 = (cy - h / 2.0) * y_factor
        x2 = (cx + w / 2.0) * x_factor
        y2 = (cy + h / 2.0) * y_factor
        boxes.append([x1, y1, max(1.0, x2 - x1), max(1.0, y2 - y1)])
        scores.append(score)
        class_ids.append(class_id)

    if not boxes:
        return []

    raw_indices = cv2.dnn.NMSBoxes(boxes, scores, float(conf_thres), float(iou_thres))
    indices: list[int] = []
    for item in raw_indices:
        if isinstance(item, (list, tuple, np.ndarray)):
            indices.append(int(item[0]))
        else:
            indices.append(int(item))

    detections: list[dict[str, Any]] = []
    for index in indices:
        x, y, w, h = boxes[index]
        x1 = max(0.0, x)
        y1 = max(0.0, y)
        x2 = min(float(image_w), x + w)
        y2 = min(float(image_h), y + h)
        class_id = class_ids[index]
        detections.append(
            {
                "bbox": [x1, y1, x2, y2],
                "center": [float((x1 + x2) * 0.5), float((y1 + y2) * 0.5)],
                "score": float(scores[index]),
                "class_name": names[class_id] if 0 <= class_id < len(names) else str(class_id),
                "depth_m": 0.0,
            }
        )
    detections.sort(key=lambda item: item.get("score", 0.0), reverse=True)
    return detections


def detect_frame_targets(
    frame,
    *,
    pipeline: str,
    target_class: str = "",
    conf_thres: float = 0.5,
    iou_thres: float = 0.45,
    hsv_range: list[int] | None = None,
    model_path: str = "",
) -> list[dict[str, Any]]:
    normalized = str(pipeline or "").strip().lower() or "yolo"
    hsv_range = list(hsv_range or [])

    if normalized in ("hsv", "color"):
        return detect_hsv_targets(
            frame,
            hsv_range=hsv_range,
            target_class=target_class,
        )

    if normalized in ("hybrid", "auto"):
        if len(hsv_range) == 6:
            hsv_hits = detect_hsv_targets(
                frame,
                hsv_range=hsv_range,
                target_class=target_class,
            )
            if hsv_hits:
                return hsv_hits
        if model_path:
            return detect_yolo_targets(
                frame,
                model_path=model_path,
                target_class=target_class,
                conf_thres=conf_thres,
                iou_thres=iou_thres,
            )
        return []

    if normalized in ("yolo", "detect", "depth", "click", "pixel"):
        if not model_path:
            return []
        return detect_yolo_targets(
            frame,
            model_path=model_path,
            target_class=target_class,
            conf_thres=conf_thres,
            iou_thres=iou_thres,
        )

    return []


def _normalize_yolo_output(output) -> np.ndarray:
    predictions = np.asarray(output)
    if predictions.ndim == 3:
        predictions = predictions[0]
    if predictions.ndim != 2:
        raise ValueError(f"unsupported YOLO output shape: {predictions.shape}")
    if predictions.shape[0] < predictions.shape[1] and predictions.shape[0] <= 128:
        predictions = predictions.transpose()
    return predictions
