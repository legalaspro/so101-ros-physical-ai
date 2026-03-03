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


"""ZeroMQ-based transport for async policy inference.

Uses a single REQ/REP socket for all communication:
- Handshake + policy config (setup phase)
- ``infer(obs) → actions`` request-response (inference phase)

Serialization: msgpack header + raw tensor bytes (no pickle).

Dependencies: pyzmq, msgpack
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import asdict

import msgpack
import numpy as np
import torch
import zmq

from lerobot.async_inference.helpers import RemotePolicyConfig, TimedAction, TimedObservation

from .base import PolicyTransport

logger = logging.getLogger("zmq_transport")


# ---------------------------------------------------------------------------
# Wire format helpers
# ---------------------------------------------------------------------------


def _serialize_observation(obs: TimedObservation) -> bytes:
    """Serialize a TimedObservation to wire format: header_len(4B) + msgpack_header + raw_array_bytes.

    The observation dict values can be numpy arrays or scalars.
    We serialize each array as raw bytes and record shape/dtype in the header.
    """
    raw_obs = obs.get_observation()

    # Separate numpy arrays from scalar/string values
    array_entries: list[tuple[str, bytes, list, str]] = []
    scalar_entries: dict[str, object] = {}

    for key, value in raw_obs.items():
        if isinstance(value, np.ndarray):
            array_entries.append((key, value.tobytes(), list(value.shape), str(value.dtype)))
        else:
            # Scalars, strings, lists — msgpack-safe
            scalar_entries[key] = value

    header = {
        "type": "observation",
        "timestep": obs.get_timestep(),
        "timestamp": obs.get_timestamp(),
        "must_go": obs.must_go,
        "arrays": {entry[0]: {"shape": entry[2], "dtype": entry[3]} for entry in array_entries},
        "scalars": scalar_entries,
    }

    header_bytes = msgpack.packb(header, use_bin_type=True)
    header_len = struct.pack("<I", len(header_bytes))

    # Concatenate all array bytes in key order matching header["arrays"]
    array_bytes = b"".join(entry[1] for entry in array_entries)

    return header_len + header_bytes + array_bytes


def _deserialize_actions(data: bytes) -> list[TimedAction]:
    """Deserialize wire bytes into a list[TimedAction].

    Wire format: header_len(4B) + msgpack_header + raw_tensor_bytes
    """
    header_len = struct.unpack("<I", data[:4])[0]
    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    actions: list[TimedAction] = []
    offset = 0

    for action_meta in header["actions"]:
        shape = tuple(action_meta["shape"])
        dtype = np.dtype(action_meta["dtype"])
        nbytes = int(np.prod(shape)) * dtype.itemsize
        arr = np.frombuffer(body[offset : offset + nbytes], dtype=dtype).reshape(shape)
        offset += nbytes

        tensor = torch.from_numpy(arr.copy())
        actions.append(
            TimedAction(
                timestamp=action_meta["timestamp"],
                timestep=action_meta["timestep"],
                action=tensor,
            )
        )

    return actions


def _serialize_policy_config(config: RemotePolicyConfig) -> bytes:
    """Serialize RemotePolicyConfig to msgpack bytes (no pickle)."""
    d = asdict(config)
    # Convert PolicyFeature dataclass values in lerobot_features to plain dicts
    lf = {}
    for k, v in d["lerobot_features"].items():
        if hasattr(v, "__dict__"):
            lf[k] = vars(v) if not isinstance(v, dict) else v
        elif isinstance(v, dict):
            lf[k] = v
        else:
            lf[k] = {"value": v}
    d["lerobot_features"] = lf
    return msgpack.packb(d, use_bin_type=True)


class ZmqTransport(PolicyTransport):
    """Client-side ZeroMQ transport implementing the PolicyTransport ABC.

    Uses a single REQ/REP socket for handshake, policy config, and
    ``infer()`` request-response calls.

    Args:
        host: Server hostname or IP.
        port: Port number for the REQ/REP socket.
        recv_timeout_ms: Timeout in ms for receiving replies (-1 = block forever).
    """

    def __init__(
        self,
        host: str = "localhost",
        port: int = 5555,
        recv_timeout_ms: int = 2000,
        logger=None,
    ):
        self._host = host
        self._port = int(port)
        self._recv_timeout_ms = recv_timeout_ms
        self._log = logger or logging.getLogger(__name__)

        self._ctx: zmq.Context | None = None
        self._req: zmq.Socket | None = None  # REQ for all communication
        self._timestep_counter = 0

    # ------------------------------------------------------------------
    # PolicyTransport interface
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Create ZMQ context and connect the REQ socket."""
        try:
            self._ctx = zmq.Context()
            self._req = self._ctx.socket(zmq.REQ)
            self._req.connect(f"tcp://{self._host}:{self._port}")
            self._log.info(f"ZMQ connected to {self._host}:{self._port}")
            return True
        except zmq.ZMQError as exc:
            self._log.error(f"ZMQ connect failed: {exc}")
            return False

    def close(self) -> None:
        """Close the socket and terminate context."""
        if self._req is not None:
            self._req.close(linger=0)
        if self._ctx is not None:
            self._ctx.term()
        self._req = self._ctx = None
        self._log.info("ZMQ transport closed")

    def handshake(self) -> bool:
        """Send a handshake request and wait for server acknowledgement."""
        try:
            t0 = time.perf_counter()
            self._req.send(msgpack.packb({"type": "handshake"}, use_bin_type=True))
            reply = msgpack.unpackb(self._req.recv(), raw=False)
            ms = (time.perf_counter() - t0) * 1000
            ok = reply.get("status") == "ok"
            if ok:
                self._log.info(f"📡 Handshake OK | {ms:.1f} ms")
            else:
                self._log.warning(f"Handshake failed: {reply}")
            return ok
        except zmq.ZMQError as exc:
            self._log.error(f"Handshake error: {exc}")
            return False

    def send_policy_config(self, config: RemotePolicyConfig) -> bool:
        """Send policy config over REQ/REP socket."""
        try:
            t0 = time.perf_counter()
            payload = _serialize_policy_config(config)
            self._req.send(msgpack.packb({"type": "policy_config", "data": payload}, use_bin_type=True))
            reply = msgpack.unpackb(self._req.recv(), raw=False)
            ms = (time.perf_counter() - t0) * 1000
            ok = reply.get("status") == "ok"
            if ok:
                self._log.info(f"📡 Policy config sent | {ms:.1f} ms | {len(payload)} bytes")
            else:
                self._log.warning(f"Policy config rejected: {reply}")
            return ok
        except zmq.ZMQError as exc:
            self._log.error(f"send_policy_config error: {exc}")
            return False

    def send_observation(self, obs: TimedObservation) -> bool:
        """Not supported — use :meth:`infer` instead."""
        raise NotImplementedError("Use infer() instead")

    def receive_actions(self) -> list[TimedAction]:
        """Not supported — use :meth:`infer` instead."""
        raise NotImplementedError("Use infer() instead")

    def infer(self, obs: TimedObservation) -> list[TimedAction]:
        """Send an observation and receive actions in a single REQ/REP round-trip."""
        self._timestep_counter += 1
        try:
            # Serialize observation
            t_start = time.perf_counter()
            obs_data = _serialize_observation(obs)
            t_serialized = time.perf_counter()
            serialize_ms = (t_serialized - t_start) * 1000

            # Send request and receive reply (round-trip)
            self._req.send(msgpack.packb({"type": "infer", "data": obs_data}, use_bin_type=True))
            reply_raw = self._req.recv()
            t_recv = time.perf_counter()
            round_trip_ms = (t_recv - t_serialized) * 1000

            # Deserialize reply
            reply = msgpack.unpackb(reply_raw, raw=False)
            if reply.get("status") != "ok":
                self._log.error(f"infer() server error: {reply.get('message', 'unknown')}")
                return []

            actions_data = reply["actions"]
            if isinstance(actions_data, bytes):
                actions = _deserialize_actions(actions_data)
            else:
                actions = _deserialize_actions(bytes(actions_data))
            t_deserialized = time.perf_counter()
            deserialize_ms = (t_deserialized - t_recv) * 1000
            total_ms = (t_deserialized - t_start) * 1000

            self._log.info(
                f"📤📥 INFER #{obs.get_timestep()} | "
                f"serialize={serialize_ms:.1f}ms | round_trip={round_trip_ms:.1f}ms | "
                f"deserialize={deserialize_ms:.1f}ms | total={total_ms:.1f}ms | "
                f"sent={len(obs_data)} bytes | recv={len(reply_raw)} bytes"
            )
            return actions
        except zmq.ZMQError as exc:
            self._log.error(f"infer() error: {exc}")
            return []
