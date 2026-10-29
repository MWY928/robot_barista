"""Project command transport for the downloaded Dobot V4 SDK."""

import time

from dobot_sdk.core.connection import DobotConnection


class CommandConnection(DobotConnection):
    """Keep SDK command formatting while handling partial TCP writes/replies."""

    MAX_REPLY_BYTES = 65536

    def send_receive_text(self, text, recv_size=1024):
        with self._lock:
            if not self._connected or self._socket is None:
                raise ConnectionError("V4 command connection is not connected.")
            command_socket = self._socket
            original_timeout = command_socket.gettimeout()
            deadline = None if original_timeout is None else time.monotonic() + original_timeout
            reply = bytearray()
            try:
                command_socket.sendall((text.rstrip("\n") + "\n").encode("utf-8"))
                while b";" not in reply:
                    if deadline is not None:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError("Timed out waiting for a complete V4 command reply.")
                        command_socket.settimeout(remaining)
                    chunk = command_socket.recv(recv_size)
                    if not chunk:
                        raise ConnectionError("V4 command connection closed before a complete reply arrived.")
                    reply.extend(chunk)
                    if len(reply) > self.MAX_REPLY_BYTES:
                        raise RuntimeError("V4 command reply exceeded 65536 bytes.")
                response, _, trailing = reply.partition(b";")
                if trailing.strip():
                    raise RuntimeError("Unexpected data after the V4 command reply.")
                return (response + b";").decode("utf-8").strip()
            except (Exception, KeyboardInterrupt):
                # After an incomplete exchange, a late reply must never be
                # consumed as the acknowledgement for another command.
                self._connected = False
                raise
            finally:
                command_socket.settimeout(original_timeout)

    def disconnect(self):
        command_socket = self._socket
        try:
            super().disconnect()
        finally:
            # The SDK skips close() if shutdown() raises (e.g. failed connect).
            if command_socket is not None:
                command_socket.close()
