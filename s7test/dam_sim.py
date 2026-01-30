#!/usr/bin/env python3
# chmod +x * to make this script executable

import zmq
import json
import time
import sys
import argparse
from datetime import datetime

# this script publishes fake dam servo positions to the message bus
# first run ot-sim with
#           sudo docker run -it --rm --name ot-test -p 102:102 -p 1234:1234 -p 5678:5678 ot-sim hivemind Procfile.single
# then run this script
# if it's working, you should see the OT-sim ouptut show that the tag that we publish is being updated and the s7 server is 
# reflecting that update

# todo: check that's actually happening by making a script that outputs the s7 memory


class MsgBusPublisher:
    def __init__(self, endpoint="tcp://127.0.0.1:1234", sender_id="test-simulator"):
        self.endpoint = endpoint
        self.sender_id = sender_id
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        print(f"Connecting to message bus at {endpoint}...")
        self.socket.connect(endpoint)
        time.sleep(0.5)

    def publish_status(self, measurements):
        msg = {
            "version": "v1",
            "kind": "Status",
            "metadata": {
                "sender": self.sender_id
            },
            "contents": {
                "measurements": measurements
            }
        }

        topic = "RUNTIME"

        # send as two ZMQ frames (topic + JSON payload)
        try:
            self.socket.send_string(topic, flags=zmq.SNDMORE | zmq.NOBLOCK)
            self.socket.send_string(json.dumps(msg), flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        print(f"[{datetime.now().strftime('%H:%M:%S')}] Published: {measurements}")

    def publish_point(self, tag, value):
        measurements = [
            {
                "tag": tag,
                "value": float(value),
                "ts": int(time.time() * 1000)
            }
        ]
        self.publish_status(measurements)

    def close(self):
        self.socket.close()
        self.context.term()


def open_dam(publisher):
    print("\n!!! Simulating Dam Opening !!!\n")

    steps = [
        ("Initial state: Dam closed", {
            "dam.servo.position": 0.0
        }),
        ("Opening: 25%", {
            "dam.servo.position": 25.0
        }),
        ("Opening: 50%", {
            "dam.servo.position": 50.0
        }),
        ("Opening: 75%", {
            "dam.servo.position": 75.0
        }),
        ("Fully open: 100%", {
            "dam.servo.position": 100.0
        }),
        ("Closing: 25%", {
            "dam.servo.position": 75.0
        }),
        ("Closing: 50%", {
            "dam.servo.position": 50.0
        }),
        ("Closing: 75%", {
            "dam.servo.position": 25.0
        }),
        ("Dam closed: 100%", {
            "dam.servo.position": 0.0
        }),
    ]

    while 1==1:
        for description, point in steps:
            print(f"\n{description}")
            publisher.publish_point("dam.servo.position", point["dam.servo.position"])
            time.sleep(2)

def main():
    parser = argparse.ArgumentParser(description="Message Bus Test Publisher")
    parser.add_argument(
        "--endpoint",
        default="tcp://127.0.0.1:1234",
        help="Message bus PUSH endpoint (default: tcp://127.0.0.1:1234)"
    )

    args = parser.parse_args()

    publisher = MsgBusPublisher(endpoint=args.endpoint)

    try:
        open_dam(publisher)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        publisher.close()
        print("\nDisconnected from message bus")


if __name__ == "__main__":
    main()
