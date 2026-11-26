import importlib.util
from pathlib import Path
import shlex
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

ROOT = Path(__file__).parent


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DdsTransportTest(unittest.TestCase):
    def test_both_entrypoints_select_the_same_transport(self):
        environments = []
        for name in ("static_session", "manual_session"):
            module = load(name)
            command = Mock()
            hooks = module.Hooks(SimpleNamespace(command=command), None, Mock(), Path("/tmp/results"))
            hooks.native_start("test-token")
            arguments = command.call_args[0][1]
            self.assertEqual(arguments[arguments.index("--ulimit") + 1], "rtprio=50")
            self.assertIn("nice=30:30", arguments)
            self.assertNotIn("SYS_NICE", arguments)
            self.assertNotIn("--privileged", arguments)
            environment = dict(arguments[index + 1].split("=", 1)
                               for index, argument in enumerate(arguments) if argument == "-e")
            for key, value in {
                "ROS_DOMAIN_ID": "89", "ROS_LOCALHOST_ONLY": "0",
                "ROS_AUTOMATIC_DISCOVERY_RANGE": "SYSTEM_DEFAULT",
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "RMW_FASTRTPS_PUBLICATION_MODE": "SYNCHRONOUS",
                "FASTRTPS_DEFAULT_PROFILES_FILE":
                    "/work/agv_workspace/integration/mapping/config/dds_loopback.xml",
            }.items():
                self.assertEqual(environment[key], value)
            environments.append({key: value for key, value in environment.items()
                                 if not key.startswith("FEEDBOT_")})
        self.assertEqual(environments[0], environments[1])

    def test_standalone_entrypoint_uses_session_transport(self):
        arguments = shlex.split((ROOT / "run.sh").read_text().replace("\\\n", ""))
        environment = dict(arguments[index + 1].split("=", 1)
                           for index, argument in enumerate(arguments) if argument == "-e")
        session_arguments = load("static_session").DDS_ENVIRONMENT
        expected = dict(session_arguments[index + 1].split("=", 1)
                        for index in range(0, len(session_arguments), 2))
        self.assertEqual(environment, expected)

    def test_xml_excludes_shared_memory_and_external_interfaces(self):
        namespace = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        root = ET.parse(ROOT / "config/dds_loopback.xml").getroot()
        transports = root.findall("dds:transport_descriptors/dds:transport_descriptor", namespace)
        self.assertEqual(len(transports), 1)
        transport = transports[0]
        self.assertEqual(transport.findtext("dds:type", namespaces=namespace), "UDPv4")
        self.assertEqual([item.text for item in transport.findall("dds:interfaceWhiteList/dds:address", namespace)],
                         ["127.0.0.1"])
        participant = root.find("dds:participant", namespace)
        self.assertEqual(participant.get("is_default_profile"), "true")
        rtps = participant.find("dds:rtps", namespace)
        self.assertEqual(rtps.findtext("dds:useBuiltinTransports", namespaces=namespace), "false")
        self.assertEqual(rtps.findtext("dds:userTransports/dds:transport_id", namespaces=namespace),
                         transport.findtext("dds:transport_id", namespaces=namespace))
        self.assertEqual([item.text for item in rtps.findall(".//dds:address", namespace)],
                         ["127.0.0.1", "127.0.0.1"])


if __name__ == "__main__":
    unittest.main()
