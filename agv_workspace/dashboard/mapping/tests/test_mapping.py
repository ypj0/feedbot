import io
import math
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from car_bridge import Runtime, serialize_ranges, store_scan_payload, transform_point, world_to_map_cell
from event_log import EventLog


class Value:
    pass


def transform(x, y, yaw):
    value = Value(); value.translation = Value(); value.rotation = Value()
    value.translation.x=x; value.translation.y=y; value.rotation.x=0; value.rotation.y=0
    value.rotation.z=math.sin(yaw/2); value.rotation.w=math.cos(yaw/2)
    return value


class GeometryTests(unittest.TestCase):
    def test_transform_geometry(self):
        point = transform_point(1, 0, transform(2, 3, math.pi / 2))
        self.assertAlmostEqual(point[0], 2); self.assertAlmostEqual(point[1], 4)

    def test_map_origin_yaw(self):
        cell = world_to_map_cell(1, 3, 1, 1, math.pi / 2, .5)
        self.assertAlmostEqual(cell[0], 4); self.assertAlmostEqual(cell[1], 0)

    def test_nonfinite_ranges_are_not_serialized(self):
        scan=Value(); scan.ranges=[1, float('nan'), float('inf'), .05, 3]
        scan.range_min=.1; scan.range_max=2; scan.angle_min=0; scan.angle_increment=.1
        self.assertEqual(len(serialize_ranges(scan, transform(0,0,0))), 1)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        event_log = EventLog(io.StringIO())
        self.addCleanup(event_log.close)
        self.commands=[]; self.runtime=Runtime(lambda *value:self.commands.append(value), event_log)
        self.runtime.update('joints', {'left':0,'right':0}); self.runtime.update('odom', {'vx':0,'wz':0})
        self.runtime.update('diagnostics', {'level':0,'values':{'initialized':'1','emergency':'0','charge':'0','left_alarm':'0','right_alarm':'0','shutdown':'0'}})

    def test_mapping_readiness_and_stale_data_not_returned(self):
        self.assertTrue(self.runtime.snapshot()['ready']); self.assertFalse(self.runtime.snapshot()['mapping_ready'])
        now=time.monotonic(); self.runtime.map_payload={'received':now-6,'value':{'data':[0]}}
        self.assertEqual(self.runtime.mapping_snapshot()['map']['data'], [0])

    def test_scan_keeps_original_receive_time(self):
        received = time.monotonic() - 0.6
        store_scan_payload(self.runtime, {'points': []}, received)
        self.assertEqual(self.runtime.scan_payload['received'], received)
        self.assertIsNone(self.runtime.mapping_snapshot()['scan'])

    def test_stop_allowed_without_mapping(self):
        control=self.runtime.snapshot()['control']
        result=self.runtime.command({'epoch':control['epoch'],'sequence':1,'action':'stop','issued_at':0})
        self.assertEqual(self.commands[-1], (0,0)); self.assertFalse(result['mapping_ready'])

    def test_original_pulse_behavior(self):
        now=self.runtime.snapshot()['sampled_at']; control=self.runtime.snapshot()['control']
        self.runtime.command({'epoch':control['epoch'],'sequence':1,'action':'forward','issued_at':now,'duration_s':.5})
        self.assertEqual(self.commands[-1], (.05,0)); self.runtime.control.velocity(self.runtime.control.deadline, True)
        self.assertEqual(self.runtime.control.action, 'stop')


if __name__ == '__main__': unittest.main()
