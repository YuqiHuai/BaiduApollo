#!/usr/bin/env python3

"""HTTP bridge for selected Apollo Cyber services."""

import argparse
import itertools
import json
import threading
import time
from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer
from socketserver import ThreadingMixIn

from google.protobuf import json_format

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.external_command_msgs import action_command_pb2
from modules.common_msgs.external_command_msgs import command_status_pb2
from modules.common_msgs.external_command_msgs import lane_follow_command_pb2
from modules.common_msgs.external_command_msgs import valet_parking_command_pb2


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9091
DEFAULT_ACTION_SERVICE_NAME = "/apollo/external_command/action"
DEFAULT_LANE_FOLLOW_SERVICE_NAME = "/apollo/external_command/lane_follow"
DEFAULT_VALET_PARKING_SERVICE_NAME = "/apollo/external_command/valet_parking"
MAX_BODY_BYTES = 1024 * 1024


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class ExternalCommandService:
    """Converts JSON requests into external-command Cyber service calls."""

    def __init__(
        self, node, service_name, command_type, command_name, sequence_numbers
    ):
        self._client = node.create_client(
            service_name,
            command_type,
            command_status_pb2.CommandStatus,
        )
        self._command_type = command_type
        self._command_name = command_name
        self._sequence_numbers = sequence_numbers
        self._lock = threading.Lock()

    def send(self, payload):
        command = self._command_type()
        json_format.ParseDict(payload, command)
        self._populate_header(command)

        if not command.IsInitialized():
            missing_fields = ", ".join(command.FindInitializationErrors())
            raise ValueError("missing required field(s): {}".format(missing_fields))

        print("{} request:\n{}".format(self._command_name, command), flush=True)

        # The Cyber Python client is synchronous. Serialize calls so a shared
        # service client is not used concurrently by HTTP worker threads.
        with self._lock:
            response = self._client.send_request(command)

        if response is None:
            raise ServiceUnavailableError(
                "{} service did not return a response".format(self._command_name)
            )

        print("{} response:\n{}".format(self._command_name, response), flush=True)

        return json_format.MessageToDict(
            response, preserving_proto_field_name=True
        )

    def _populate_header(self, command):
        sequence_number = next(self._sequence_numbers)
        command.header.sequence_num = sequence_number
        command.header.timestamp_sec = time.time()
        command.header.module_name = "service_bridge"
        if not command.HasField("command_id"):
            command.command_id = sequence_number


class ServiceUnavailableError(RuntimeError):
    pass


def make_handler(services):
    class ServiceBridgeHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != "/healthz":
                self._write_json(404, {"ok": False, "error": "not found"})
                return

            self._write_json(200, {"ok": True})

        def do_POST(self):
            service = services.get(self.path)
            if service is None:
                self._write_json(404, {"ok": False, "error": "not found"})
                return

            try:
                payload = self._read_json_body()
                response = service.send(payload)
            except ServiceUnavailableError as error:
                self._write_json(503, {"ok": False, "error": str(error)})
                return
            except (
                UnicodeDecodeError,
                json.JSONDecodeError,
                json_format.ParseError,
                ValueError,
            ) as error:
                self._write_json(400, {"ok": False, "error": str(error)})
                return

            self._write_json(200, {"ok": True, "response": response})

        def log_message(self, message_format, *args):
            print(
                "{} - {}".format(self.address_string(), message_format % args),
                flush=True,
            )

        def _read_json_body(self):
            content_length = self.headers.get("Content-Length")
            if content_length is None:
                raise ValueError("Content-Length header is required")

            try:
                body_size = int(content_length)
            except ValueError as error:
                raise ValueError("invalid Content-Length header") from error

            if body_size < 0 or body_size > MAX_BODY_BYTES:
                raise ValueError(
                    "request body must be between 0 and {} bytes".format(
                        MAX_BODY_BYTES
                    )
                )

            payload = json.loads(self.rfile.read(body_size).decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("request body must be a JSON object")
            return payload

        def _write_json(self, status_code, payload):
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(status_code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return ServiceBridgeHandler


def parse_args():
    parser = argparse.ArgumentParser(
        description="Expose selected external-command Cyber services over HTTP JSON."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--action-service-name", default=DEFAULT_ACTION_SERVICE_NAME)
    parser.add_argument(
        "--lane-follow-service-name", default=DEFAULT_LANE_FOLLOW_SERVICE_NAME
    )
    parser.add_argument(
        "--valet-parking-service-name",
        default=DEFAULT_VALET_PARKING_SERVICE_NAME,
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cyber.init("service_bridge")
    node = cyber.Node("service_bridge")
    sequence_numbers = itertools.count(1)
    services = {
        "/action": ExternalCommandService(
            node,
            args.action_service_name,
            action_command_pb2.ActionCommand,
            "action",
            sequence_numbers,
        ),
        "/lane_follow": ExternalCommandService(
            node,
            args.lane_follow_service_name,
            lane_follow_command_pb2.LaneFollowCommand,
            "lane_follow",
            sequence_numbers,
        ),
        "/valet_parking": ExternalCommandService(
            node,
            args.valet_parking_service_name,
            valet_parking_command_pb2.ValetParkingCommand,
            "valet_parking",
            sequence_numbers,
        ),
    }
    server = ThreadingHTTPServer((args.host, args.port), make_handler(services))

    print(
        "service_bridge listening on http://{}:{} for {}".format(
            args.host, args.port, ", ".join(sorted(services))
        ),
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        cyber.shutdown()


if __name__ == "__main__":
    main()
