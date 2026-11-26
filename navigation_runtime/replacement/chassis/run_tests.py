#!/usr/bin/env python3
"""Disconnected tests only: creates Unix sockets and PTYs, never opens device paths."""
import pathlib, platform, subprocess, tempfile
root = pathlib.Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix='feedbot-chassis-tests-') as build:
    for name in ('test', 'transport_test', 'pty_test', 'stationary_probe', 'odometry_test', 'stationarity_test'):
        binary = str(pathlib.Path(build) / name)
        command = ['clang++', '-std=c++17', '-O1', '-g', '-Wall', '-Wextra',
                   '-fsanitize=address,undefined', str(root / (name + '.cpp')),
                   '-pthread', '-o', binary]
        if name == 'pty_test' and platform.system() == 'Linux':
            command.append('-lutil')
        subprocess.run(command, check=True)
        if name=='stationary_probe':
            subprocess.run(['python3',str(root/'probe_test.py'),binary],check=True)
        else:
            subprocess.run([binary], check=True)
