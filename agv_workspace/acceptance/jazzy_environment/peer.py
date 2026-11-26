import json
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
from std_msgs.msg import String


def main():
    role, output = sys.argv[1:]
    if role not in ("talker", "listener"):
        raise ValueError("role must be talker or listener")
    rclpy.init()
    node = Node("environment_" + role)
    received = 0
    sent = 0
    responder_counts = {}
    transmit_topic = "/environment_probe/request" if role == "talker" else "/environment_probe/reply"
    receive_topic = "/environment_probe/reply" if role == "talker" else "/environment_probe/request"
    publisher = node.create_publisher(String, transmit_topic, 10)

    def receive(message):
        nonlocal received, sent
        received += 1
        if role == "listener":
            publisher.publish(String(data=str(os.getpid()) + ":" + message.data))
            sent += 1
        else:
            responder, sequence = message.data.split(":")
            if not 0 <= int(sequence) < sent:
                raise ValueError("reply does not correspond to a transmitted request")
            responder_counts[responder] = responder_counts.get(responder, 0) + 1

    def transmit():
        nonlocal sent
        publisher.publish(String(data=str(sent)))
        sent += 1

    node.create_subscription(String, receive_topic, receive, 10)
    if role == "talker":
        node.create_timer(0.1, transmit)
    deadline = time.monotonic() + (26 if role == "talker" else 8)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = {
        "role": role,
        "pid": os.getpid(),
        "rmw": get_rmw_implementation_identifier(),
        "sent": sent,
        "received": received,
        "responder_counts": responder_counts,
    }
    node.destroy_node()
    rclpy.shutdown()
    Path(output).write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
