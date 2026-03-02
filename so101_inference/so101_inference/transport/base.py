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

"""Abstract base class for policy transport layers."""

from __future__ import annotations

from abc import ABC, abstractmethod

from lerobot.async_inference.helpers import RemotePolicyConfig, TimedAction, TimedObservation


class PolicyTransport(ABC):
    """Abstract base class defining the transport interface between robot client and policy server.

    Implementations handle the network communication details (gRPC, ZeroMQ, etc.)
    while exposing a uniform API for the async inference pipeline.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Establish connection to the policy server.

        Returns:
            True if connection was established successfully, False otherwise.
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """Close the connection and release resources."""
        ...

    @abstractmethod
    def handshake(self) -> bool:
        """Perform initial handshake with the server.

        Returns:
            True if handshake succeeded, False otherwise.
        """
        ...

    @abstractmethod
    def send_policy_config(self, config: RemotePolicyConfig) -> bool:
        """Send policy configuration to the server.

        Args:
            config: The remote policy configuration to send.

        Returns:
            True if config was accepted by the server, False otherwise.
        """
        ...

    @abstractmethod
    def send_observation(self, obs: TimedObservation) -> bool:
        """Send an observation to the server for inference.

        Args:
            obs: The timed observation to send.

        Returns:
            True if observation was sent successfully, False otherwise.
        """
        ...

    @abstractmethod
    def receive_actions(self) -> list[TimedAction]:
        """Receive action chunks from the server.

        Returns:
            List of timed actions from the server. Empty list if no actions available.
        """
        ...

    @abstractmethod
    def infer(self, obs: TimedObservation) -> list[TimedAction]:
        """Send an observation and receive actions in a single blocking round-trip.

        This is the preferred single-call API.  Transports that support a native
        request-response pattern (e.g. ZeroMQ REQ/REP) implement this directly.
        Others (e.g. gRPC) may simply delegate to ``send_observation`` +
        ``receive_actions``.

        Args:
            obs: The timed observation to send.

        Returns:
            List of timed actions from the server.
        """
        ...
