#!/usr/bin/env python3
"""Independent chassis SSH via the RK's loopback relay and ADB port forwarding."""
import pathlib,subprocess,sys
home=pathlib.Path.home()
args=['ssh','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=feedbot-onboard','-o','UserKnownHostsFile='+str(home/'.ssh/feedbot_onboard_known_hosts'),'-i',str(home/'.ssh/feedbot_native_ed25519'),'-p','22088','root@127.0.0.1']
raise SystemExit(subprocess.call(args+sys.argv[1:]))
