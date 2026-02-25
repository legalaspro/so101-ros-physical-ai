# Copyright 2026 Dmitri Manajev
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

import numpy as np
from sensor_msgs.msg import Image


def ros2_image_to_numpy(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to numpy array (H, W, 3) uint8 RGB."""
    # Direct memory view → numpy, zero-copy
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()

    if enc not in ("rgb8", "bgr8"):
        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)

    if msg.encoding == "bgr8":
        img = img[:, :, ::-1].copy()  # BGR → RGB, need copy for contiguous memory
    elif msg.encoding == "rgb8":
        img = np.ascontiguousarray(img)  # ensure contiguous

    return img
