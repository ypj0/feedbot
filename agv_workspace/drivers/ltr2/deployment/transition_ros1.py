#!/usr/bin/env python
"""Call a managed driver transition and propagate its actual result."""

import sys

import rospy
from std_srvs.srv import Trigger


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("configure", "activate", "deactivate", "cleanup"):
        raise SystemExit("usage: transition_ros1.py configure|activate|deactivate|cleanup")
    rospy.init_node("ltr2_transition_client", anonymous=True, disable_signals=True)
    name = "/ltr2_driver/" + sys.argv[1]
    rospy.wait_for_service(name, timeout=5)
    response = rospy.ServiceProxy(name, Trigger)()
    print(response.message)
    return 0 if response.success else 1


if __name__ == "__main__":
    sys.exit(main())
