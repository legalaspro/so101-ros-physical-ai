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

"""Unified CLI entry point for the policy server.

Usage::

    python -m policy_server.server --transport=grpc --host=0.0.0.0 --port=8090 --fps=50
    python -m policy_server.server --transport=zmq  --host=0.0.0.0 --port=8090 --fps=50
"""

from __future__ import annotations

import argparse
import logging
import os
from concurrent import futures
from dataclasses import asdict
from pprint import pformat

from policy_server.inference_engine import InferenceEngine, InferenceEngineConfig


def _serve_grpc(args: argparse.Namespace) -> None:
    """Start the gRPC policy server."""
    import grpc

    from lerobot.async_inference.configs import PolicyServerConfig
    from lerobot.transport import services_pb2_grpc  # type: ignore
    from policy_server.grpc_server import PolicyServer

    cfg = PolicyServerConfig(
        host=args.host,
        port=args.port,
        fps=args.fps,
        inference_latency=args.inference_latency,
        obs_queue_timeout=args.obs_queue_timeout,
    )

    logging.info(pformat(asdict(cfg)))

    engine = InferenceEngine(
        InferenceEngineConfig(
            fps=args.fps,
            inference_latency=args.inference_latency,
            obs_queue_timeout=args.obs_queue_timeout,
        )
    )

    policy_server = PolicyServer(cfg, engine=engine)

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    services_pb2_grpc.add_AsyncInferenceServicer_to_server(policy_server, server)
    server.add_insecure_port(f"{args.host}:{args.port}")

    logging.info(f"gRPC PolicyServer started on {args.host}:{args.port}")
    server.start()
    server.wait_for_termination()
    logging.info("gRPC server terminated")


def _serve_zmq(args: argparse.Namespace) -> None:
    """Start the ZMQ policy server."""
    from policy_server.zmq_server import ZmqPolicyServer, ZmqServerConfig

    config = ZmqServerConfig(
        host=args.host,
        port=args.port,
        fps=args.fps,
        inference_latency=args.inference_latency,
        obs_queue_timeout=args.obs_queue_timeout,
    )
    server = ZmqPolicyServer(config)
    server.serve()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Unified policy server (gRPC or ZMQ transport)",
    )
    parser.add_argument(
        "--transport",
        choices=["grpc", "zmq"],
        default="grpc",
        help="Transport protocol (default: grpc)",
    )
    parser.add_argument("--host", default="0.0.0.0", help="Host address to bind to")
    parser.add_argument("--port", type=int, default=8080, help="Port number")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    parser.add_argument(
        "--inference-latency",
        type=float,
        default=0.033,
        help="Target inference latency in seconds",
    )
    parser.add_argument(
        "--obs-queue-timeout",
        type=float,
        default=2.0,
        help="Observation queue timeout in seconds",
    )

    args = parser.parse_args()

    # Enable DEBUG on console if requested via LOGLEVEL env var
    log_level = os.environ.get("LOGLEVEL", "INFO").upper()
    logging.basicConfig(level=log_level)

    if args.transport == "grpc":
        _serve_grpc(args)
    else:
        _serve_zmq(args)


if __name__ == "__main__":
    main()
