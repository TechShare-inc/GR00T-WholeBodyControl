"""Small request/reply client for the simulator-only WBC reset handshake."""

from __future__ import annotations

import uuid

import zmq


class SimulationResetClient:
    """Request a resident WBC reset without exposing this path to hardware."""

    def __init__(self, host: str = "127.0.0.1", port: int = 0, timeout_ms: int = 1000):
        self.endpoint = f"tcp://{host}:{int(port)}"
        self.timeout_ms = int(timeout_ms)
        self._active_request_id: str | None = None

    def _request(self, operation: str, request_id: str) -> bool:
        if not self.endpoint or self.endpoint.endswith(":0"):
            return False

        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        try:
            socket.connect(self.endpoint)
            socket.send_json(
                {
                    "version": 1,
                    "command": operation,
                    "request_id": request_id,
                }
            )
            response = socket.recv_json()
            return response.get("status") == "accepted"
        except (zmq.ZMQError, ValueError, TypeError):
            return False
        finally:
            socket.close(0)
            context.term()

    def prepare(self) -> bool:
        """Stop policy output and wait until the WBC reset hold is active."""
        request_id = str(uuid.uuid4())
        accepted = self._request("simulation_reset_prepare", request_id)
        self._active_request_id = request_id if accepted else None
        return accepted

    def complete(self) -> bool:
        """Tell WBC that MuJoCo has been restored to its neutral data state."""
        if self._active_request_id is None:
            return False
        accepted = self._request("simulation_reset_complete", self._active_request_id)
        self._active_request_id = None
        return accepted
