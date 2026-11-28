#!/usr/bin/env python3
"""Feedbot IMU CBOR 解码 + 日志 + 批次间隔统计。

纯标准库实现，无 paho-mqtt / cbor2 依赖，适用于 RK3588 (Python 3.6)。
订阅 imu_probe/<client_id>/axis，解码 CBOR 遥测帧，写 JSON lines 日志，
同时打印每条批次的到达间隔与 smp_ts_ms 间隔，用于验证发布频率和超时阈值。
"""

import json
import os
import socket
import struct
import sys
import time

BROKER_HOST = '127.0.0.1'
BROKER_PORT = 1883
CLIENT_ID = 'rk_feedbot_imu_logger'
TOPIC = 'imu_probe/288485026D38/axis'
LOG_PATH = '/opt/feedbot-imu-log/imu_axis.log'
LOG_DIR = os.path.dirname(LOG_PATH)


def cbor_decode(data):
    """解码 IMU axis CBOR map（5 键），只支持本固件使用的类型。"""
    idx = [0]

    def read_u8():
        b = data[idx[0]]
        idx[0] += 1
        return b

    def read_bytes(k):
        b = data[idx[0]:idx[0] + k]
        idx[0] += k
        return b

    def read_uint(additional):
        if additional < 24:
            return additional
        if additional == 24:
            return read_u8()
        if additional == 25:
            return struct.unpack('>H', read_bytes(2))[0]
        if additional == 26:
            return struct.unpack('>I', read_bytes(4))[0]
        if additional == 27:
            return struct.unpack('>Q', read_bytes(8))[0]
        raise ValueError('bad uint additional %d' % additional)

    def read_head():
        b = read_u8()
        return (b >> 5) & 0x07, b & 0x1f

    def read_value(major, additional):
        if major == 0:
            return read_uint(additional)
        if major == 3:
            length = read_uint(additional)
            return read_bytes(length).decode('utf-8', 'replace')
        if major == 4:
            length = read_uint(additional)
            out = []
            for _ in range(length):
                m2, a2 = read_head()
                out.append(read_value(m2, a2))
            return out
        if major == 7:
            if additional == 26:
                return struct.unpack('>f', read_bytes(4))[0]
            if additional == 27:
                return struct.unpack('>d', read_bytes(8))[0]
            raise ValueError('bad simple additional %d' % additional)
        raise ValueError('unsupported major %d' % major)

    major, additional = read_head()
    if major != 5:
        raise ValueError('top-level is not map, major=%d' % major)
    count = read_uint(additional)
    result = {}
    for _ in range(count):
        km, ka = read_head()
        key = read_value(km, ka)
        vm, va = read_head()
        result[key] = read_value(vm, va)
    return result


def encode_remaining_length(length):
    out = bytearray()
    while True:
        digit = length % 128
        length //= 128
        if length:
            digit |= 0x80
        out.append(digit)
        if not length:
            break
    return bytes(out)


def mqtt_packet(packet_type, flags, payload):
    header = bytes([packet_type | flags])
    return header + encode_remaining_length(len(payload)) + payload


def mqtt_connect(sock):
    client_id = CLIENT_ID.encode()
    payload = struct.pack('>H', len(client_id)) + client_id
    variable = struct.pack('>H', 4) + b'MQTT' + bytes([4, 0x02]) + struct.pack('>H', 60)
    sock.sendall(mqtt_packet(0x10, 0, variable + payload))


def mqtt_subscribe(sock, topic):
    packet_id = 1
    t = topic.encode()
    payload = struct.pack('>H', packet_id) + struct.pack('>H', len(t)) + t + b'\x00'
    sock.sendall(mqtt_packet(0x82, 0x02, payload))


def read_exact(sock, n):
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError('socket closed')
        data += chunk
    return data


def read_mqtt_packet(sock):
    b0 = read_exact(sock, 1)[0]
    packet_type = b0 >> 4
    multiplier = 1
    remaining = 0
    while True:
        b = read_exact(sock, 1)[0]
        remaining += (b & 0x7f) * multiplier
        if not (b & 0x80):
            break
        multiplier *= 128
    return packet_type, read_exact(sock, remaining)


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    sock = socket.create_connection((BROKER_HOST, BROKER_PORT), timeout=10)
    sock.settimeout(35)
    mqtt_connect(sock)
    ptype, payload = read_mqtt_packet(sock)
    print('CONNACK type=%d %s' % (ptype, payload.hex()), flush=True)
    mqtt_subscribe(sock, TOPIC)
    ptype, payload = read_mqtt_packet(sock)
    print('SUBACK type=%d %s' % (ptype, payload.hex()), flush=True)
    print('subscribed %s, waiting for frames...' % TOPIC, flush=True)

    last_recv = None
    last_smp = None
    count = 0
    last_ping = time.time()

    while True:
        try:
            ptype, payload = read_mqtt_packet(sock)
        except socket.timeout:
            if time.time() - last_ping > 30:
                sock.sendall(b'\xc0\x00')
                last_ping = time.time()
            continue

        if ptype == 0x0D:
            continue
        if ptype != 0x03:
            print('unexpected packet type %d' % ptype, flush=True)
            continue

        tlen = struct.unpack('>H', payload[:2])[0]
        topic = payload[2:2 + tlen].decode('utf-8', 'replace')
        data = payload[2 + tlen:]
        now = time.time()

        try:
            frame = cbor_decode(data)
        except Exception as error:
            print('decode error: %s head=%s' % (error, data[:16].hex()), flush=True)
            continue

        smp_ts_ms = frame.get('smp_ts_ms')
        axis_mask = frame.get('axis_mask')
        chn = frame.get('chn')
        batch_size = frame.get('batch_size')
        samples = frame.get('batch_samples') or []

        interval = (now - last_recv) if last_recv is not None else None
        smp_interval = (smp_ts_ms - last_smp) if (last_smp is not None and smp_ts_ms is not None) else None

        record = {
            'recv_epoch': round(now, 3),
            'recv_iso': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(now))
                        + '.%03d' % int((now % 1) * 1000),
            'smp_ts_ms': smp_ts_ms,
            'axis_mask': axis_mask,
            'chn': chn,
            'batch_size': batch_size,
            'sample_count': len(samples),
            'roll_deg': round(samples[0], 4) if samples else None,
            'roll_vel_rad_s': round(samples[1], 4) if len(samples) > 1 else None,
            'batch_interval_s': round(interval, 3) if interval is not None else None,
            'smp_interval_ms': smp_interval,
        }

        with open(LOG_PATH, 'a') as f:
            f.write(json.dumps(record, ensure_ascii=False) + '\n')

        count += 1
        last_recv = now
        last_smp = smp_ts_ms

        roll = record['roll_deg'] if record['roll_deg'] is not None else float('nan')
        vel = record['roll_vel_rad_s'] if record['roll_vel_rad_s'] is not None else float('nan')
        print('[%d] %s mask=0x%03X chn=%s batch=%s samples=%d roll=%.3f vel=%.3f '
              'batch_int=%.3fs smp_int=%sms'
              % (count, record['recv_iso'], axis_mask or 0, chn, batch_size,
                 len(samples), roll, vel,
                 interval if interval is not None else float('nan'),
                 smp_interval if smp_interval is not None else 'nan'), flush=True)


if __name__ == '__main__':
    main()
