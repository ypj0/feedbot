#!/usr/bin/env python3

import binascii
import json
import socket
import subprocess
import sys
import threading


QUERY_METHODS = ["device/queryInfo", "settings/get", "settings/get", "settings/get"]
INITIAL_STOP_METHODS = ["scan/stopStreaming", "scan/stopMeasurement"]
CYCLE_METHODS = [
    "scan/startMeasurement",
    "scan/startStreaming",
    "scan/stopStreaming",
    "scan/stopMeasurement",
]


def decode_request(line):
    header, body = line.split(b",0:,", 1)
    length, framed_payload = body.split(b":", 1)
    payload_length = int(length)
    if payload_length > 65535 or framed_payload[payload_length:] != b",\r\n":
        raise ValueError("invalid request length framing")
    payload = framed_payload[:payload_length]
    if not header.startswith(b"15:checksum=0x"):
        raise ValueError("missing checksum header")
    expected = int(header.split(b"0x", 1)[1], 16)
    if binascii.crc_hqx(payload, 65535) != expected:
        raise ValueError("request checksum mismatch")
    request = json.loads(payload)
    if request.get("jsonrpc") != "2.0" or not isinstance(request.get("id"), int):
        raise ValueError("invalid JSON-RPC request")
    return request


def encode_response(document):
    payload = json.dumps(document, separators=(",", ":")).encode()
    checksum = binascii.crc_hqx(payload, 65535)
    return ("15:checksum=0x%04X,0:,%d:" % (checksum, len(payload))).encode() + payload + b",\r\n"


def result_for(request, scenario):
    entry = request.get("params", {}).get("entry")
    values = {
        "identity.model": "LT-R2" if scenario == "wrong_identity" else "LT-R2-B",
        "transport.oob.enabled": True,
        "transport.oob.targetAddress": "127.0.0.1",
        "transport.oob.targetPort": 2105,
    }
    if request["method"] in ("device/queryInfo", "settings/get"):
        return values[entry]
    if request["method"] in CYCLE_METHODS:
        return True
    raise ValueError("unexpected method " + request["method"])


def serve(server, scenario, calls, errors):
    try:
        connection, peer = server.accept()
        if peer[0] != "127.0.0.1":
            raise ValueError("non-loopback peer")
        connection.settimeout(3)
        with connection, connection.makefile("rb") as stream:
            while True:
                line = stream.readline(65538)
                if not line:
                    return
                if len(line) > 65537:
                    raise ValueError("request line too large")
                request = decode_request(line)
                method = request["method"]
                calls.append(method)
                if scenario == "malformed" and len(calls) == 1:
                    connection.sendall(b"{invalid-json}\r\n")
                    continue
                if scenario == "timeout" and method == "scan/startStreaming":
                    continue
                error_method = {
                    "rpc_error": "scan/startStreaming",
                    "initial_streaming_stop_error": "scan/stopStreaming",
                    "initial_measurement_stop_error": "scan/stopMeasurement",
                }.get(scenario)
                if method == error_method:
                    response = {
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "error": {"code": -32603, "message": "fixture failure"},
                    }
                else:
                    response = {
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": result_for(request, scenario),
                    }
                connection.sendall(encode_response(response))
                if scenario == "peer_close" and calls == QUERY_METHODS:
                    return
    except Exception as error:
        errors.append(repr(error))


def run_case(binary, scenario):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    server.settimeout(3)
    calls = []
    errors = []
    worker = threading.Thread(target=serve, args=(server, scenario, calls, errors))
    worker.start()
    try:
        result = subprocess.run(
            [binary, str(server.getsockname()[1])],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=10,
            check=False,
        )
    finally:
        worker.join(4)
        server.close()
    if worker.is_alive():
        raise AssertionError(scenario + " fixture server did not exit")
    if errors:
        raise AssertionError(scenario + " server errors: " + repr(errors))
    expected_success = scenario == "normal"
    expected_exit = 0 if expected_success else 2
    if result.returncode != expected_exit:
        raise AssertionError(
            "%s unexpected exit %d stdout=%r stderr=%r"
            % (scenario, result.returncode, result.stdout, result.stderr)
        )
    expected_calls = QUERY_METHODS + INITIAL_STOP_METHODS + CYCLE_METHODS + CYCLE_METHODS
    if scenario == "normal" and calls != expected_calls:
        raise AssertionError("normal command order mismatch: " + repr(calls))
    if scenario == "wrong_identity" and calls != QUERY_METHODS:
        raise AssertionError("wrong identity issued writes: " + repr(calls))
    if scenario == "malformed" and calls != ["device/queryInfo"]:
        raise AssertionError("malformed response did not bound validation: " + repr(calls))
    initial_stop_failure = QUERY_METHODS + INITIAL_STOP_METHODS
    if scenario in ("initial_streaming_stop_error", "initial_measurement_stop_error"):
        if calls != initial_stop_failure:
            raise AssertionError(scenario + " did not attempt both initial stops: " + repr(calls))
    failure_prefix = (QUERY_METHODS + INITIAL_STOP_METHODS +
                      ["scan/startMeasurement", "scan/startStreaming"])
    if scenario in ("rpc_error", "timeout"):
        if calls != failure_prefix + ["scan/stopStreaming", "scan/stopMeasurement"]:
            raise AssertionError(scenario + " cleanup order mismatch: " + repr(calls))
    if scenario == "peer_close" and calls != QUERY_METHODS:
        raise AssertionError("peer-close validation mismatch: " + repr(calls))
    return {
        "scenario": scenario,
        "exit": result.returncode,
        "calls": calls,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
    }


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_sdk_control.py CONTROL_FIXTURE")
    reports = [
        run_case(sys.argv[1], scenario)
        for scenario in (
            "normal",
            "wrong_identity",
            "malformed",
            "initial_streaming_stop_error",
            "initial_measurement_stop_error",
            "rpc_error",
            "timeout",
            "peer_close",
        )
    ]
    print(json.dumps(reports, indent=2))


if __name__ == "__main__":
    main()
