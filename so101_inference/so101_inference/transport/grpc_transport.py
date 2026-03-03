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

"""gRPC implementation of PolicyTransport."""

from __future__ import annotations

import logging
import pickle  # nosec
import time

import grpc

from lerobot.async_inference.helpers import RemotePolicyConfig, TimedAction, TimedObservation
from lerobot.transport import services_pb2  # type: ignore
from lerobot.transport import services_pb2_grpc  # type: ignore
from lerobot.transport.utils import grpc_channel_options, send_bytes_in_chunks

from so101_inference.transport.base import PolicyTransport

log = logging.getLogger(__name__)


class GrpcTransport(PolicyTransport):
    """gRPC-based transport for communicating with a remote policy server.

    Wraps the existing gRPC+pickle protocol: channel creation, stub calls,
    chunked observation sending, and action deserialization.

    Args:
        server_address: ``host:port`` of the policy server.
        fps: Target FPS, used to derive the gRPC initial-backoff interval.
    """

    def __init__(self, server_address: str, fps: float = 30.0, logger=None) -> None:
        self._server_address = server_address
        self._fps = fps
        self._log = logger or logging.getLogger(__name__)
        self._channel: grpc.Channel | None = None
        self._stub: services_pb2_grpc.AsyncInferenceStub | None = None

    # ------------------------------------------------------------------
    # PolicyTransport interface
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            self._channel = grpc.insecure_channel(
                self._server_address,
                grpc_channel_options(initial_backoff=f"{1.0 / self._fps:.4f}s"),
            )
            self._stub = services_pb2_grpc.AsyncInferenceStub(self._channel)
            return True
        except Exception as exc:
            self._log.error(f"Failed to create gRPC channel: {exc}")
            return False

    def close(self) -> None:
        if self._channel is not None:
            try:
                self._channel.close()
            except Exception:
                pass
            self._channel = None
            self._stub = None

    def handshake(self) -> bool:
        try:
            t0 = time.perf_counter()
            self._stub.Ready(services_pb2.Empty())
            ms = (time.perf_counter() - t0) * 1000
            self._log.info(f"📡 Server handshake OK | {ms:.1f} ms")
            return True
        except grpc.RpcError as e:
            self._log.error(f"gRPC handshake failed: {e}")
            return False

    def send_policy_config(self, config: RemotePolicyConfig) -> bool:
        try:
            data = pickle.dumps(config)
            t0 = time.perf_counter()
            self._stub.SendPolicyInstructions(services_pb2.PolicySetup(data=data))
            ms = (time.perf_counter() - t0) * 1000
            self._log.info(f"📡 Policy config sent | {ms:.1f} ms | {len(data)} bytes")
            return True
        except grpc.RpcError as e:
            self._log.error(f"gRPC send_policy_config failed: {e}")
            return False

    def send_observation(self, obs: TimedObservation) -> bool:
        try:
            t0 = time.perf_counter()
            payload = pickle.dumps(obs)
            serialize_ms = (time.perf_counter() - t0) * 1000

            t1 = time.perf_counter()
            iterator = send_bytes_in_chunks(
                payload,
                services_pb2.Observation,
                log_prefix="[ROS2 CLIENT] Observation",
                silent=True,
            )
            _ = self._stub.SendObservations(iterator)
            send_ms = (time.perf_counter() - t1) * 1000

            self._log.info(
                f"📤 OBS #{obs.get_timestep()} SENT | must_go={obs.must_go}"
                f" | serialize={serialize_ms:.1f}ms | send={send_ms:.1f}ms | {len(payload)} bytes"
            )
            return True
        except grpc.RpcError as e:
            self._log.error(f"SendObservations FAILED for obs #{obs.get_timestep()}: {e}")
            return False

    def receive_actions(self) -> list[TimedAction]:
        try:
            rpc_start = time.perf_counter()
            chunk = self._stub.GetActions(services_pb2.Empty())
            rpc_ms = (time.perf_counter() - rpc_start) * 1000

            if len(chunk.data) == 0:
                self._log.debug(f"GetActions returned empty | rpc={rpc_ms:.1f}ms")
                return []

            deser_start = time.perf_counter()
            timed_actions: list[TimedAction] = pickle.loads(chunk.data)  # nosec
            deser_ms = (time.perf_counter() - deser_start) * 1000

            if not timed_actions:
                self._log.debug("GetActions deserialized to empty list")
                return []

            self._log.info(
                f"📥 CHUNK RECEIVED | actions={len(timed_actions)} | rpc={rpc_ms:.1f}ms | deser={deser_ms:.1f}ms"
            )
            return timed_actions
        except grpc.RpcError as e:
            self._log.error(f"GetActions RPC ERROR: {e}")
            raise

    def infer(self, obs: TimedObservation) -> list[TimedAction]:
        """Send an observation and receive actions (wrapper over send + receive)."""
        self.send_observation(obs)
        return self.receive_actions()
