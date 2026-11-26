import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time


RELEASE = Path('/opt/feedbot-integration/releases/20261123-base-v14-ekf-async')
LOCALIZATION = Path('/opt/feedbot-localization-verifier/releases/20261123-v2-source-epoch')
LIFECYCLE = Path('/opt/feedbot-localization-lifecycle/releases/20261123-v5-acquisition')
SOURCE = Path('/var/tmp/feedbot-imu-latest78-source')
TOOLS = Path(__file__).resolve().parent / 'localization_tools'
TOOL_CONTAINER = 'feedbot-localization-tools'
ROOT = Path('/var/tmp/feedbot-ekf85-live')


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


trial = load('layers', RELEASE / 'agv_workspace/acceptance/runs/20261123_base_lifecycle_35/check_layers.py')
trial.ROOT = ROOT
ownership = load('ownership', RELEASE / 'agv_workspace/integration/bringup/ownership.py')


def container(name, evidence, script, detached=False):
    return trial.DOCKER + ['run'] + (['--detach', '--init'] if detached else []) + ['--rm', '--name', name, '--network', 'host', '--ipc', 'host'] + \
        ownership.DDS_ENVIRONMENT + ['-v', str(RELEASE) + ':/work:ro',
        '-v', str(LOCALIZATION) + ':/localization:ro', '-v', str(SOURCE) + ':/trial:ro',
        '-v', str(TOOLS) + ':/tools:ro', '-v', str(evidence) + ':/evidence', '--entrypoint', '/bin/bash',
        'feedbot-navigation:jazzy-20261122', '-c', 'source /opt/ros/jazzy/setup.bash; ' + script]


def run_container(name, evidence, script, timeout, tools_id):
    return trial.command(name, trial.DOCKER + [
        'exec', '--workdir', '/evidence/' + evidence.name, tools_id,
        '/bin/bash', '-c', 'source /opt/ros/jazzy/setup.bash; ' + script], timeout)


def main():
    for path in (TOOLS / 'acquire_seed.py', SOURCE / 'record.py', TOOLS / 'converge.py',
                 SOURCE / 'initialize.py',
                 LOCALIZATION / 'start.sh', LIFECYCLE / 'start_lifecycle.sh'):
        if not path.is_file():
            raise RuntimeError('missing deployed input: ' + str(path))
    ROOT.mkdir(exist_ok=False)
    report = dict(passed=False, motion_commands_sent=False, activation_attempts=0,
                  initialization_attempts=0, convergence_attempts=0)
    invocation = None
    services = []
    recorder = None
    record_log = None
    tools_id = None
    try:
        tools_id = trial.command('start_tools', container(TOOL_CONTAINER, ROOT,
                                 'exec sleep infinity', detached=True), 15).strip()
        report['activation_attempts'] = 1
        trial.command('start_base', ['bash', str(RELEASE / 'agv_workspace/integration/bringup/run.sh'),
                                    str(ROOT / 'base')], 10)
        identity = trial.command('base_identity', ['systemctl', 'show', 'feedbot-base',
                                 '-p', 'InvocationID', '-p', 'ExecStart'], 10)
        properties = dict(line.split('=', 1) for line in identity.splitlines() if '=' in line)
        expected = str(RELEASE / 'agv_workspace/integration/bringup/session.py') + ' ' + str(ROOT / 'base') + ' ;'
        if expected not in properties.get('ExecStart', '') or not properties.get('InvocationID'):
            raise RuntimeError('base session ownership not established')
        invocation = properties['InvocationID']
        report['base_invocation_id'] = invocation
        trial.wait_for_file(ROOT / 'base/sessionready.json', 120)
        for directory in ('record', 'seed', 'convergence'):
            (ROOT / directory).mkdir()
        record_log = (ROOT / 'record.log').open('w')
        recorder = subprocess.Popen(container('feedbot-localization70-record', ROOT / 'record',
                    'exec python3 -B /trial/record.py'), stdout=record_log, stderr=subprocess.STDOUT)
        trial.wait_for_file(ROOT / 'record/status.json', 10)
        report['localization_start_monotonic'] = time.monotonic()
        for service, argv in (
                ('feedbot-map-layer', ['bash', str(RELEASE / 'agv_workspace/integration/mapping/start.sh'),
                 'localization', '/opt/feedbot-maps/20261122T222225Z_7e73c2f0/feedbot.yaml']),
                ('feedbot-localization-verifier', ['bash', str(LOCALIZATION / 'start.sh')]),
                ('feedbot-localization-lifecycle', ['bash', str(LIFECYCLE / 'start_lifecycle.sh')])):
            trial.command('start_' + service, argv, 10)
            services.append(service)
        run_container('feedbot-localization70-seed', ROOT / 'seed',
                      'exec python3 -B /tools/acquire_seed.py', 30, tools_id)
        seed = json.loads((ROOT / 'seed/seed.json').read_text())
        if time.monotonic() - seed['created_monotonic'] > 3:
            raise RuntimeError('candidate expired before initialization')
        report['initialization_attempts'] = 1
        (ROOT / 'convergence/initialization.json').write_text(json.dumps(
            dict(before_initialization_ns=int(time.time() * 1000000000))) + '\n')
        run_container('feedbot-localization70-initialize', ROOT / 'seed',
            'exec python3 -B /trial/initialize.py '
            '/evidence/seed/seed.json', 15, tools_id)
        report['convergence_attempts'] = 1
        run_container('feedbot-localization70-converge', ROOT / 'convergence',
                      'exec python3 -B /tools/converge.py', 50, tools_id)
        trial.command('final_probe', trial.DOCKER + ['exec', 'feedbot-base', '/bin/bash', '-c',
            'source /opt/ros/jazzy/setup.bash; python3 -B '
            '/work/agv_workspace/integration/bringup/probe.py /results/final_probe.json 5'], 30)
        report['observation_completed'] = True
    except BaseException as error:
        report['error'] = '%s: %s' % (type(error).__name__, error)
    finally:
        for service in reversed(services):
            try:
                trial.command(service + '_journal', ['journalctl', '-u', service, '--no-pager', '-n', '120'], 10)
            except BaseException as error:
                report[service + '_journal_error'] = str(error)
            try:
                trial.command('stop_' + service, ['systemctl', 'stop', service], 45)
            except BaseException as error:
                report[service + '_cleanup_error'] = str(error)
        if recorder is not None:
            try:
                if recorder.poll() is None:
                    trial.command('stop_record', trial.DOCKER + ['stop', '-t', '5', 'feedbot-localization70-record'], 10)
                report['recorder_exit'] = recorder.wait(timeout=10)
            except BaseException as error:
                report['recorder_cleanup_error'] = str(error)
        if tools_id:
            try:
                trial.command('stop_tools', trial.DOCKER + ['stop', '-t', '5', tools_id], 10)
            except BaseException as error:
                report['tools_cleanup_error'] = str(error)
        try:
            if invocation and not (ROOT / 'base/session.json').exists():
                current = trial.command('base_identity_before_stop', ['systemctl', 'show', 'feedbot-base',
                                        '-p', 'InvocationID', '--value'], 10).strip()
                if current != invocation:
                    raise RuntimeError('base instance changed')
                trial.command('stop_base', ['systemctl', 'stop', 'feedbot-base'], 180)
            session = json.loads((ROOT / 'base/session.json').read_text())
            report['factory_restored'] = session['factory_restored']
            report['base_session_passed'] = session['passed']
        except BaseException as error:
            report['base_cleanup_error'] = str(error)
        if record_log is not None:
            record_log.close()
        report['passed'] = bool(report.get('observation_completed') and report.get('factory_restored')
                                and report.get('base_session_passed') and report.get('recorder_exit') == 0
                                and not any(key.endswith('error') for key in report))
        (ROOT / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
