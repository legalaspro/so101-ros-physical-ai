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

"""ZeroMQ policy server — wraps the existing PolicyServer inference logic.

Uses a single REP socket for all communication:
- Handshake + policy config (setup phase)
- ``infer`` request-response (inference phase)

Serialization: msgpack header + raw tensor bytes (no pickle).

Dependencies: pyzmq, msgpack

Usage:
    python -m policy_server.zmq_server --host 0.0.0.0 --port 8090 --fps 50
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass

import msgpack
import numpy as np
import zmq

from lerobot.async_inference.helpers import RemotePolicyConfig, TimedAction, TimedObservation, get_logger
from policy_server.inference_engine import InferenceEngine, InferenceEngineConfig

logger = get_logger("zmq_server", log_to_file=False)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass
class ZmqServerConfig:
    host: str = "0.0.0.0"
    port: int = 5555
    fps: int = 30
    inference_latency: float = 0.033
    obs_queue_timeout: float = 2.0

    @property
    def environment_dt(self) -> float:
        return 1.0 / self.fps


# ---------------------------------------------------------------------------
# Wire-format helpers (mirror zmq_transport.py)
# ---------------------------------------------------------------------------


def _deserialize_observation(data: bytes) -> TimedObservation:
    """Deserialize wire bytes into a TimedObservation."""
    header_len = struct.unpack("<I", data[:4])[0]
    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    raw_obs: dict = dict(header.get("scalars", {}))

    offset = 0
    for key, meta in header.get("arrays", {}).items():
        shape = tuple(meta["shape"])
        dtype = np.dtype(meta["dtype"])
        nbytes = int(np.prod(shape)) * dtype.itemsize
        arr = np.frombuffer(body[offset : offset + nbytes], dtype=dtype).reshape(shape)
        offset += nbytes
        raw_obs[key] = arr  # keep as numpy; conversion happens downstream

    return TimedObservation(
        timestamp=header["timestamp"],
        timestep=header["timestep"],
        observation=raw_obs,
        must_go=header.get("must_go", False),
    )


def _serialize_actions(actions: list[TimedAction]) -> bytes:
    """Serialize a list[TimedAction] to wire format."""
    action_metas = []
    tensor_parts: list[bytes] = []

    for ta in actions:
        arr = ta.get_action().detach().cpu().numpy()
        action_metas.append(
            {
                "timestamp": ta.get_timestamp(),
                "timestep": ta.get_timestep(),
                "shape": list(arr.shape),
                "dtype": str(arr.dtype),
            }
        )
        tensor_parts.append(arr.tobytes())

    header = {"type": "actions", "actions": action_metas}
    header_bytes = msgpack.packb(header, use_bin_type=True)
    header_len = struct.pack("<I", len(header_bytes))

    return header_len + header_bytes + b"".join(tensor_parts)


def _deserialize_policy_config(payload: bytes) -> RemotePolicyConfig:
    """Deserialize msgpack bytes into a RemotePolicyConfig."""
    d = msgpack.unpackb(payload, raw=False)
    # lerobot_features values are plain dicts (with keys: dtype, shape, names).
    # They're used as-is by build_dataset_frame, raw_observation_to_observation,
    # make_lerobot_observation, etc. — no conversion to PolicyFeature needed.
    return RemotePolicyConfig(**d)


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------


class ZmqPolicyServer:
    """ZeroMQ policy server — delegates inference to :class:`InferenceEngine`.

    Lifecycle:
        1. Client sends handshake (REQ/REP)
        2. Client sends policy config (REQ/REP) → server loads model
        3. Client sends ``infer`` requests (REQ/REP) → server runs inference inline
    """

    def __init__(self, config: ZmqServerConfig | None = None):
        self.config = config or ZmqServerConfig()
        self.engine = InferenceEngine(
            InferenceEngineConfig(
                fps=self.config.fps,
                inference_latency=self.config.inference_latency,
                obs_queue_timeout=self.config.obs_queue_timeout,
            )
        )

    @property
    def running(self) -> bool:
        return self.engine.running

    # ------------------------------------------------------------------
    # ZMQ event loop
    # ------------------------------------------------------------------

    def _handle_req_rep(self, rep_socket: zmq.Socket) -> None:
        """Handle all REQ/REP messages (handshake, policy config, infer) in a loop."""
        while self.running:
            try:
                msg = rep_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.01)
                continue

            request = msgpack.unpackb(msg, raw=False)
            msg_type = request.get("type")

            if msg_type == "handshake":
                logger.info("Handshake received")
                self.engine.reset()
                self.engine.resume()
                rep_socket.send(msgpack.packb({"status": "ok"}, use_bin_type=True))

            elif msg_type == "policy_config":
                try:
                    config = _deserialize_policy_config(request["data"])
                    self.engine.load_policy(config)
                    rep_socket.send(msgpack.packb({"status": "ok"}, use_bin_type=True))
                except Exception as exc:
                    logger.error(f"Failed to load policy: {exc}")
                    rep_socket.send(
                        msgpack.packb({"status": "error", "message": str(exc)}, use_bin_type=True)
                    )

            elif msg_type == "infer":
                try:
                    loop_start = time.perf_counter()
                    receive_time = time.time()

                    # Deserialize observation
                    obs_data = request["data"]
                    if not isinstance(obs_data, bytes):
                        obs_data = bytes(obs_data)
                    obs = _deserialize_observation(obs_data)

                    obs_timestep = obs.get_timestep()
                    obs_timestamp = obs.get_timestamp()

                    fps_metrics = self.engine.fps_tracker.calculate_fps_metrics(obs_timestamp)

                    logger.info(
                        f"Received observation #{obs_timestep} | "
                        f"Avg FPS: {fps_metrics['avg_fps']:.2f} | "
                        f"One-way latency: {(receive_time - obs_timestamp) * 1000:.2f}ms"
                    )

                    # Run inference directly (no queue) — replicate run_inference() logic
                    with self.engine._predicted_timesteps_lock:
                        self.engine._predicted_timesteps.add(obs_timestep)

                    inference_start = time.perf_counter()
                    action_chunk = self.engine._predict_action_chunk(obs)
                    inference_ms = (time.perf_counter() - inference_start) * 1000

                    # Serialize actions
                    serialize_start = time.perf_counter()
                    actions_data = _serialize_actions(action_chunk)
                    serialize_ms = (time.perf_counter() - serialize_start) * 1000

                    total_ms = (time.perf_counter() - loop_start) * 1000

                    logger.info(
                        f"Action chunk generated | "
                        f"Total time: {total_ms:.2f}ms | "
                        f"inference={inference_ms:.1f}ms | "
                        f"serialize={serialize_ms:.1f}ms | "
                        f"{len(actions_data)} bytes"
                    )

                    rep_socket.send(
                        msgpack.packb(
                            {"status": "ok", "actions": actions_data},
                            use_bin_type=True,
                        )
                    )
                except Exception as exc:
                    logger.error(f"Infer error: {exc}", exc_info=True)
                    rep_socket.send(
                        msgpack.packb({"status": "error", "message": str(exc)}, use_bin_type=True)
                    )

            else:
                logger.warning(f"Unknown request type: {msg_type}")
                rep_socket.send(
                    msgpack.packb({"status": "error", "message": "unknown type"}, use_bin_type=True)
                )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def serve(self) -> None:
        """Start the ZMQ server (blocking)."""
        ctx = zmq.Context()

        rep = ctx.socket(zmq.REP)
        rep.bind(f"tcp://{self.config.host}:{self.config.port}")

        logger.info(f"ZmqPolicyServer listening on {self.config.host}:{self.config.port}")

        thread = threading.Thread(target=self._handle_req_rep, args=(rep,), daemon=True)
        thread.start()

        try:
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            self.engine.shutdown_event.set()
            thread.join(timeout=2)
            rep.close(linger=0)
            ctx.term()
            logger.info("ZMQ server stopped")

    def stop(self) -> None:
        """Signal the server to stop."""
        self.engine.stop()


def main():
    """CLI entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="ZMQ Policy Server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--inference-latency", type=float, default=0.033)
    parser.add_argument("--obs-queue-timeout", type=float, default=2.0)
    args = parser.parse_args()

    config = ZmqServerConfig(
        host=args.host,
        port=args.port,
        fps=args.fps,
        inference_latency=args.inference_latency,
        obs_queue_timeout=args.obs_queue_timeout,
    )
    server = ZmqPolicyServer(config)
    server.serve()


if __name__ == "__main__":
    main()
