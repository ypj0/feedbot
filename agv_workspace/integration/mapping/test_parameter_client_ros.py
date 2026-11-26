import unittest

import rclpy
from rclpy.parameter import Parameter

from static_probe import read_system_time_parameter


class ParameterClientRuntimeTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("mapping_parameter_client_test")

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_actual_jazzy_parameter_service(self):
        self.assertEqual(read_system_time_parameter(self.node, self.node.get_fully_qualified_name()),
                         {"available": True, "value": False})
        self.node.set_parameters([Parameter("use_sim_time", value=True)])
        self.assertEqual(read_system_time_parameter(self.node, self.node.get_fully_qualified_name()),
                         {"available": True, "value": True})

    def test_missing_service_does_not_substitute_system_time(self):
        self.assertEqual(read_system_time_parameter(self.node, "/absent_parameter_server"),
                         {"available": False})


if __name__ == "__main__":
    unittest.main()
