#!/usr/bin/env python3
"""把标定结果持久化到车机 /opt/feedbot-navigation/charge_dock_config.json。

用法：从 bridge 标定返回的 JSON 读取 dock/pre_dock，写车机文件。
     echo '{"dock": {...}, "pre_dock": {...}}' | python3 persist_dock.py
或：  python3 persist_dock.py '{"dock": {...}, "pre_dock": {...}}'
"""
import json
import pathlib
import subprocess
import sys

REMOTE = '/opt/feedbot-navigation/charge_dock_config.json'
TOOL = pathlib.Path(__file__).resolve().parents[3] / 'navigation_runtime/replacement/tools/native_ssh.py'


def main():
    if len(sys.argv) > 1:
        cfg = json.loads(sys.argv[1])
    else:
        cfg = json.loads(sys.stdin.read())
    if 'dock' not in cfg or not cfg['dock']:
        print('no dock pose in input')
        sys.exit(1)
    payload = json.dumps({'dock': cfg['dock'], 'pre_dock': cfg.get('pre_dock')},
                         ensure_ascii=False) + '\n'
    p = subprocess.run(['python3', str(TOOL), 'cat > %s' % REMOTE],
                       input=payload.encode(), capture_output=True)
    if p.returncode != 0:
        print('ssh write failed:', p.stderr.decode())
        sys.exit(1)
    # 触发回充节点重新加载配置
    subprocess.run(['python3', str(TOOL),
                    '/opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock exec feedbot-navigation bash -lc "source /opt/ros/jazzy/setup.bash; ros2 service call /charge_dock/reload std_srvs/srv/Trigger \\"{}\\" >/dev/null 2>&1"'],
                   check=True)
    print('persisted to', REMOTE)
    print(json.dumps(cfg, ensure_ascii=False))


if __name__ == '__main__':
    main()
