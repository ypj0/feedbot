import math

import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer

from car_bridge import ScanPairing


def main():
    rclpy.init()
    buffer = Buffer()
    pairing = ScanPairing()
    scan = LaserScan()
    scan.header.frame_id = 'scan'
    scan.header.stamp.sec = 1
    scan.header.stamp.nanosec = 20000000
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'scan'
    transform.header.stamp.sec = 1
    transform.transform.rotation.w = 1.
    buffer.set_transform(transform, 'fixture')

    def available(message):
        return buffer.can_transform('map', message.header.frame_id,
                                    rclpy.time.Time.from_msg(message.header.stamp))

    try:
        pairing.offer(scan, 10.)
        assert pairing.take(10.01, 1030000000, available) is None
        newer = LaserScan()
        newer.header.frame_id = 'scan'
        newer.header.stamp.sec = 1
        newer.header.stamp.nanosec = 70000000
        pairing.offer(newer, 10.05)
        transform.header.stamp.nanosec = 40000000
        transform.transform.translation.x = .04
        buffer.set_transform(transform, 'fixture')
        received, selected = pairing.take(10.06, 1080000000, available)
        result = buffer.lookup_transform('map', 'scan',
                                        rclpy.time.Time.from_msg(selected.header.stamp))
        assert selected is scan and received == 10.
        assert result.header.stamp.nanosec == 20000000
        assert math.isclose(result.transform.translation.x, .02)
        pairing.offer(newer, 10.07)
        try:
            pairing.take(10.571, 1571000000, available)
        except ValueError as error:
            assert 'expired' in str(error)
        else:
            raise AssertionError('missing TF deadline was not enforced')
        print('PASS: real tf2 delayed arrival, exact-time interpolation, newer scan cannot overwrite, expiry')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
