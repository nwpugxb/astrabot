#!/usr/bin/env python3
"""TCP relay: ESP32 finds the host (client); sllidar uses localhost.

Flow (host is the server — same idea as micro-ROS AGENT_IP):
  1. Listen on device_port (20108) for ESP32.
  2. After ESP32 connects, listen on 127.0.0.1:sllidar_port (20109).
  3. Bridge bytes; log sllidar<->esp32 counts each second.
"""

from __future__ import annotations

import select
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node


class RplidarTcpRelay(Node):
    def __init__(self) -> None:
        super().__init__("rplidar_tcp_relay")
        self.declare_parameter("device_port", 20108)
        self.declare_parameter("sllidar_port", 20109)
        self.declare_parameter("bind_address", "0.0.0.0")

        self._device_port = int(self.get_parameter("device_port").value)
        self._sllidar_port = int(self.get_parameter("sllidar_port").value)
        self._bind = str(self.get_parameter("bind_address").value)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"Host server ready — waiting for ESP32 on {self._bind}:{self._device_port}"
        )

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()

    def _make_listener(self, host: str, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.listen(1)
        sock.settimeout(1.0)
        return sock

    def _accept(self, listener: socket.socket, label: str) -> Optional[socket.socket]:
        while not self._stop.is_set():
            try:
                conn, addr = listener.accept()
            except socket.timeout:
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info(f"{label} connected from {addr[0]}:{addr[1]}")
            return conn
        return None

    def _pump(self, device: socket.socket, sllidar: socket.socket) -> None:
        """device=ESP32 side, sllidar=local SDK side."""
        sockets = [device, sllidar]
        to_esp = 0
        from_esp = 0
        last_log = time.monotonic()
        while not self._stop.is_set():
            try:
                readable, _, errored = select.select(sockets, [], sockets, 0.5)
            except (ValueError, OSError):
                break
            if errored:
                break
            for src in readable:
                try:
                    data = src.recv(4096)
                except OSError:
                    return
                if not data:
                    return
                dst = sllidar if src is device else device
                try:
                    dst.sendall(data)
                except OSError:
                    return
                if src is sllidar:
                    to_esp += len(data)
                else:
                    from_esp += len(data)

            now = time.monotonic()
            if now - last_log >= 1.0:
                self.get_logger().info(
                    f"bytes sllidar→esp32={to_esp}  esp32→sllidar={from_esp}"
                )
                if to_esp > 0 and from_esp == 0:
                    self.get_logger().warn(
                        "SDK commands reach ESP32 but no lidar bytes return — "
                        "check UART TX/RX (SWAP_UART_PINS), 5V, GND, unplug A1 USB"
                    )
                to_esp = 0
                from_esp = 0
                last_log = now

    def _serve(self) -> None:
        device_listener = self._make_listener(self._bind, self._device_port)
        try:
            while not self._stop.is_set():
                device = self._accept(device_listener, "ESP32")
                if device is None:
                    break

                sllidar_listener = self._make_listener("127.0.0.1", self._sllidar_port)
                self.get_logger().info(
                    f"ESP32 online — sllidar may connect to 127.0.0.1:{self._sllidar_port}"
                )
                try:
                    sllidar = self._accept(sllidar_listener, "sllidar")
                    if sllidar is None:
                        device.close()
                        break
                    self.get_logger().info("Bridging ESP32 <-> sllidar")
                    try:
                        self._pump(device, sllidar)
                    finally:
                        self.get_logger().warn("Bridge closed; waiting for ESP32 again")
                        try:
                            sllidar.close()
                        except OSError:
                            pass
                        try:
                            device.close()
                        except OSError:
                            pass
                finally:
                    sllidar_listener.close()
        finally:
            device_listener.close()


def main() -> None:
    rclpy.init()
    node = RplidarTcpRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
