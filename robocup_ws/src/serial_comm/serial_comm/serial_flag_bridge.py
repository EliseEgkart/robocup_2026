#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import serial


def _parse_hex_bytes(s: str) -> bytes:
    tokens = s.replace(",", " ").split()
    if not tokens:
        return b""
    out = bytearray()
    for t in tokens:
        t = t.strip()
        if t.startswith(("0x", "0X")):
            t = t[2:]
        out.append(int(t, 16))
    return bytes(out)


class SerialLink:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout_s: float = 0.1,
    ):
        self._port = port
        self._baudrate = baudrate
        self._timeout_s = timeout_s
        self._ser: Optional[serial.Serial] = None

    def open(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            self._port,
            self._baudrate,
            timeout=self._timeout_s,
        )

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def send(self, data: bytes) -> None:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial port is not open")
        self._ser.write(data)

    def recv_once(self, size: int, overall_timeout_s: float = 1.0) -> bytes:
        if size <= 0:
            return b""
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial port is not open")

        buf = bytearray()
        deadline = time.time() + overall_timeout_s
        while len(buf) < size and time.time() < deadline:
            chunk = self._ser.read(size - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def recv_until(
        self,
        delimiter: bytes = b"\n",
        max_bytes: int = 4096,
        overall_timeout_s: float = 1.0,
    ) -> bytes:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial port is not open")

        buf = bytearray()
        deadline = time.time() + overall_timeout_s
        while len(buf) < max_bytes and time.time() < deadline:
            chunk = self._ser.read(1)
            if not chunk:
                continue
            buf.extend(chunk)
            if buf.endswith(delimiter):
                break
        return bytes(buf)


class SerialFlagBridge(Node):
    def __init__(self):
        super().__init__("serial_flag_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout_s", 0.1)
        self.declare_parameter("topic", "flagf")
        self.declare_parameter("msg_type", "bool")  # "bool" or "string"
        self.declare_parameter("edge_trigger", True)
        self.declare_parameter("send_on_true", True)
        self.declare_parameter("tx_hex", "")       # e.g. "AA 55 01 0D 0A"
        self.declare_parameter("tx_text", "F")     # used if tx_hex is empty
        self.declare_parameter("trigger_text", "1")  # for string messages

        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        timeout_s = self.get_parameter("timeout_s").get_parameter_value().double_value
        self._topic = self.get_parameter("topic").get_parameter_value().string_value
        self._msg_type = self.get_parameter("msg_type").get_parameter_value().string_value
        self._edge_trigger = self.get_parameter("edge_trigger").get_parameter_value().bool_value
        self._send_on_true = self.get_parameter("send_on_true").get_parameter_value().bool_value
        self._trigger_text = self.get_parameter("trigger_text").get_parameter_value().string_value

        tx_hex = self.get_parameter("tx_hex").get_parameter_value().string_value.strip()
        if tx_hex:
            self._tx_bytes = _parse_hex_bytes(tx_hex)
        else:
            tx_text = self.get_parameter("tx_text").get_parameter_value().string_value
            self._tx_bytes = tx_text.encode("utf-8")

        self._link = SerialLink(port=port, baudrate=baudrate, timeout_s=timeout_s)
        self._link.open()

        self._last_bool: Optional[bool] = None

        if self._msg_type == "string":
            self._sub = self.create_subscription(String, self._topic, self._on_string, 10)
        else:
            self._sub = self.create_subscription(Bool, self._topic, self._on_bool, 10)

        self.get_logger().info(
            f"serial_flag_bridge ready: topic={self._topic} type={self._msg_type} port={port} baud={baudrate}"
        )

    def destroy_node(self):
        try:
            self._link.close()
        finally:
            super().destroy_node()

    def _send(self) -> None:
        try:
            self._link.send(self._tx_bytes)
            self.get_logger().info(f"sent {self._tx_bytes!r}")
        except Exception as exc:
            self.get_logger().error(f"send failed: {exc}")

    def _on_bool(self, msg: Bool) -> None:
        should_send = (msg.data is True) if self._send_on_true else (msg.data is False)
        if not should_send:
            return
        if self._edge_trigger and self._last_bool is True:
            return
        self._last_bool = True
        self._send()

    def _on_string(self, msg: String) -> None:
        if msg.data != self._trigger_text:
            return
        self._send()


def main(args=None):
    rclpy.init(args=args)
    node = SerialFlagBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
