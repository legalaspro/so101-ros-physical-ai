# Copyright 2025 Dmitri Manajev
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Decoder registry for rosbag_to_lerobot.
Maps ROS message types -> numpy arrays suitable for LeRobotDataset.add_frame().
"""

from __future__ import annotations

import io
from typing import Any, Callable, Dict

import numpy as np

from rosbag_to_lerobot.config import FeatureSpec

try:
    import imageio.v3 as iio
except ImportError:  # older imageio
    import imageio as iio  # type: ignore[no-redef]


# -------------------------
# Registry
# -------------------------

DecoderFn = Callable[[Any, FeatureSpec], np.ndarray]

DECODERS: Dict[str, DecoderFn] = {}
DTYPES: Dict[str, str] = {}  # LeRobot dtype string: "video", "float32", etc.


def register_decoder(
    msg_type: str, *, lerobot_dtype: str
) -> Callable[[DecoderFn], DecoderFn]:
    """Decorator to register a decoder for a ROS msg type."""

    def _wrap(fn: DecoderFn) -> DecoderFn:
        if msg_type in DECODERS:
            raise RuntimeError(f"Decoder already registered for {msg_type}")
        DECODERS[msg_type] = fn
        DTYPES[msg_type] = lerobot_dtype
        return fn

    return _wrap


def get_decoder(msg_type: str) -> DecoderFn:
    try:
        return DECODERS[msg_type]
    except KeyError as e:
        raise KeyError(
            f"No decoder registered for msg_type='{msg_type}'. "
            f"Registered: {sorted(DECODERS.keys())}"
        ) from e


def get_lerobot_dtype(msg_type: str) -> str:
    # default float32 for vectors unless overridden by registry
    return DTYPES.get(msg_type, "float32")


def decode(msg: Any, spec: FeatureSpec) -> np.ndarray:
    """Decode message according to spec.msg_type using registry."""
    fn = get_decoder(spec.msg_type)
    return fn(msg, spec)


# -------------------------
# Helpers
# -------------------------


def _ensure_shape(arr: np.ndarray, spec: FeatureSpec) -> np.ndarray:
    """Optional sctrict shape check using spec.shape"""
    if spec.shape is None:
        return arr
    expected = tuple(int(x) for x in spec.shape)
    if arr.shape != expected:
        raise ValueError(
            f"{spec.key}: decoded shape {arr.shape} != expected {expected} "
            f"(topic={spec.topic}, msg_type={spec.msg_type})"
        )
    return arr


# -------------------------
# Decoders
# -------------------------


@register_decoder("sensor_msgs/msg/Image", lerobot_dtype="video")
def decode_image(msg: Any, spec: FeatureSpec) -> np.ndarray:
    enc = (msg.encoding or "").lower()
    h, w = int(msg.height), int(msg.width)

    if enc in ("rgb8", "bgr8"):
        ch = 3
    elif enc in ("rgba8", "bgra8"):
        ch = 4
    elif enc in ("mono8", "8uc1"):
        ch = 1
    else:
        raise ValueError(
            f"Unsupported image encoding: '{msg.encoding}' (topic={spec.topic})"
        )

    step = int(getattr(msg, "step", 0) or (w * ch))
    expected = w * ch
    if step != expected:
        raise ValueError(
            f"{spec.key}: padded rows not supported (step={step}, expected={expected}) "
            f"encoding={msg.encoding} size={h}x{w} topic={spec.topic}"
        )

    raw = np.frombuffer(memoryview(msg.data), dtype=np.uint8).reshape(h, w, ch)

    if enc == "bgr8":
        raw = raw[..., ::-1].copy()
    elif enc == "rgba8":
        raw = raw[..., :3].copy()
    elif enc == "bgra8":
        raw = raw[..., 2::-1].copy()
    elif enc in ("mono8", "8uc1"):
        raw = np.repeat(raw[:, :, None], 3, axis=2)

    return _ensure_shape(np.ascontiguousarray(raw, dtype=np.uint8), spec)


@register_decoder("sensor_msgs/msg/CompressedImage", lerobot_dtype="video")
def decode_compressed_image(msg: Any, spec: FeatureSpec) -> np.ndarray:
    """Decode sensor_msgs/msg/CompressedImage -> HWC uint8 ndarray (RGB-ish)."""
    buf = bytes(msg.data)
    img = iio.imread(io.BytesIO(buf))

    if img.ndim == 2:
        img = np.stack([img, img, img], axis=-1)
    elif img.ndim == 3 and img.shape[2] == 4:
        img = img[:, :, :3]

    img = np.ascontiguousarray(img, dtype=np.uint8)
    return _ensure_shape(img, spec)


@register_decoder("sensor_msgs/msg/JointState", lerobot_dtype="float32")
def decode_joint_state(msg: Any, spec: FeatureSpec) -> np.ndarray:
    """Decode sensor_msgs/msg/JointState -> float32 vector, optionally ordered by spec.names"""
    pos = np.asarray(msg.position, dtype=np.float32).flatten()

    if not spec.names:
        return _ensure_shape(pos, spec)

    name_to_idx = {n: i for i, n in enumerate(list(msg.name))}
    out = np.zeros((len(spec.names),), dtype=np.float32)
    for j, joint in enumerate(spec.names):
        i = name_to_idx.get(joint)
        if i is not None and i < len(pos):
            out[j] = pos[i]

    return _ensure_shape(out, spec)


@register_decoder("std_msgs/msg/Float64MultiArray", lerobot_dtype="float32")
def decode_float64_multiarray(msg: Any, spec: FeatureSpec) -> np.ndarray:
    """
    Decode Float64MultiArray.data into a vector.
    """
    arr = np.asarray(msg.data, dtype=np.float32).flatten()
    return _ensure_shape(arr, spec)
