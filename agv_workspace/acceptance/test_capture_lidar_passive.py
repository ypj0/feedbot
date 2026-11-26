import os
import socket
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from capture_lidar_passive import lidar_udp


class FilterTests(unittest.TestCase):
    def frame(self):
        frame = bytearray(58)
        frame[12:14] = b"\x08\x00"
        frame[14] = 0x45
        frame[23] = 17
        frame[26:30] = socket.inet_aton("192.168.10.160")
        struct.pack_into("!H", frame, 16, 44)
        struct.pack_into("!HHH", frame, 34, 2105, 2105, 24)
        return frame

    def test_accepts_expected_packet(self):
        self.assertTrue(lidar_udp(self.frame()))

    def test_rejects_every_truncation(self):
        frame = self.frame()
        for length in range(len(frame)):
            self.assertFalse(lidar_udp(frame[:length]))

    def test_rejects_wrong_source_port_fragment_and_bad_length(self):
        for offset, value in ((26, 10), (23, 6), (34, 0), (36, 0), (20, 0x20), (14, 0x44), (39, 1)):
            frame = self.frame()
            frame[offset] = value
            self.assertFalse(lidar_udp(frame), (offset, value))


if __name__ == "__main__":
    unittest.main()
