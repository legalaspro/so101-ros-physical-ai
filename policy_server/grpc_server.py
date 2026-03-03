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

"""Thin gRPC servicer — delegates all inference to :class:`InferenceEngine`.

This replaces the old ``policy_server.py`` which mixed transport and
inference logic.
"""

from __future__ import annotations

import logging
import os
import pickle  # nosec
import time
from concurrent import futures
from dataclasses import asdict
from pprint import pformat

import grpc

from lerobot.async_inference.configs import PolicyServerConfig
from lerobot.async_inference.helpers import (
    RemotePolicyConfig,
    get_logger,
)
from lerobot.transport import services_pb2  # type: ignore
from lerobot.transport import services_pb2_grpc  # type: ignore
from lerobot.transport.utils import receive_bytes_in_chunks

from policy_server.inference_engine import InferenceEngine, InferenceEngineConfig

logger = get_logger("grpc_server", log_to_file=False)


class PolicyServer(services_pb2_grpc.AsyncInferenceServicer):
    """gRPC servicer that wraps :class:`InferenceEngine`."""

    def __init__(self, config: PolicyServerConfig, engine: InferenceEngine | None = None) -> None:
        self.config = config
        self.engine = engine or InferenceEngine(
            InferenceEngineConfig(
                fps=config.fps,
                inference_latency=config.inference_latency,
                obs_queue_timeout=config.obs_queue_timeout,
            )
        )

    def Ready(self, request, context):  # noqa: N802
        client_id = context.peer()
        logger.info(f"Client {client_id} connected and ready")
        self.engine.reset()
        self.engine.resume()
        return services_pb2.Empty()

    def SendPolicyInstructions(self, request, context):  # noqa: N802
        if not self.engine.running:
            logger.warning("Server is not running. Ignoring policy instructions.")
            return services_pb2.Empty()

        client_id = context.peer()
        policy_specs = pickle.loads(request.data)  # nosec

        if not isinstance(policy_specs, RemotePolicyConfig):
            raise TypeError(f"Policy specs must be a RemotePolicyConfig. Got {type(policy_specs)}")

        logger.info(
            f"Receiving policy instructions from {client_id} | "
            f"Policy type: {policy_specs.policy_type} | "
            f"Pretrained name or path: {policy_specs.pretrained_name_or_path} | "
            f"Actions per chunk: {policy_specs.actions_per_chunk} | "
            f"Device: {policy_specs.device}"
        )

        self.engine.load_policy(policy_specs)
        return services_pb2.Empty()

    def SendObservations(self, request_iterator, context):  # noqa: N802
        client_id = context.peer()
        logger.debug(f"Receiving observations from {client_id}")

        receive_time = time.time()
        start_deserialize = time.perf_counter()
        received_bytes = receive_bytes_in_chunks(request_iterator, None, self.engine.shutdown_event, logger)
        timed_observation = pickle.loads(received_bytes)  # nosec
        deserialize_time = time.perf_counter() - start_deserialize

        logger.debug(f"Received observation #{timed_observation.get_timestep()}")

        obs_timestep = timed_observation.get_timestep()
        obs_timestamp = timed_observation.get_timestamp()

        fps_metrics = self.engine.fps_tracker.calculate_fps_metrics(obs_timestamp)

        logger.info(
            f"Received observation #{obs_timestep} | "
            f"Avg FPS: {fps_metrics['avg_fps']:.2f} | "
            f"Target: {fps_metrics['target_fps']:.2f} | "
            f"One-way latency: {(receive_time - obs_timestamp) * 1000:.2f}ms"
        )

        logger.debug(
            f"Server timestamp: {receive_time:.6f} | "
            f"Client timestamp: {obs_timestamp:.6f} | "
            f"Deserialization time: {deserialize_time:.6f}s"
        )

        if not self.engine.enqueue_observation(timed_observation):
            logger.debug(f"Observation #{obs_timestep} has been filtered out")

        return services_pb2.Empty()

    def GetActions(self, request, context):  # noqa: N802
        client_id = context.peer()
        logger.debug(f"Client {client_id} connected for action streaming")

        try:
            getactions_starts = time.perf_counter()
            action_chunk = self.engine.run_inference()

            if action_chunk is None:
                return services_pb2.Empty()

            start_time = time.perf_counter()
            actions_bytes = pickle.dumps(action_chunk)  # nosec
            serialize_time = time.perf_counter() - start_time

            actions = services_pb2.Actions(data=actions_bytes)

            logger.info(
                f"Action chunk generated | "
                f"Total time: {(time.perf_counter() - getactions_starts) * 1000:.2f}ms"
            )

            time.sleep(
                max(0, self.config.inference_latency - max(0, time.perf_counter() - getactions_starts))
            )

            return actions

        except Exception as e:
            logger.error(f"Error in GetActions: {e}")
            return services_pb2.Empty()

    def stop(self):
        self.engine.stop()
        logger.info("gRPC server stopping...")
