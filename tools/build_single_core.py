#!/usr/bin/env python3
"""Build SCAN-Planner on Linux with one CPU and record memory pressure."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def memory_sample():
    values = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if key in ('MemAvailable', 'SwapTotal', 'SwapFree'):
            values[key + '_KiB'] = int(value.strip().split()[0])
    values['memory_pressure'] = Path('/proc/pressure/memory').read_text().strip()
    values['time'] = datetime.datetime.now().astimezone().isoformat()
    return values


def group_alive(group):
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == group and fields[0] != 'Z':
                return True
        except (OSError, ValueError, IndexError):
            pass
    return False


def stop_group(group):
    if group_alive(group):
        os.killpg(group, signal.SIGTERM)
        deadline = time.monotonic() + 8
        while group_alive(group) and time.monotonic() < deadline:
            time.sleep(0.2)
    if group_alive(group):
        os.killpg(group, signal.SIGKILL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default='/home/marsdog/SCAN-Planner')
    parser.add_argument('--cpu', type=int, default=4)
    parser.add_argument('--with-tests', action='store_true')
    args = parser.parse_args()
    workspace = Path(args.workspace).resolve()
    if not (workspace / 'src').is_dir():
        parser.error('workspace/src does not exist')
    if args.cpu not in os.sched_getaffinity(0):
        parser.error('requested CPU is unavailable')
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    output = workspace / 'log' / ('single-core-' + stamp)
    output.mkdir(parents=True, exist_ok=False)
    command = [
        'colcon', 'build', '--symlink-install', '--executor', 'sequential',
        '--parallel-workers', '1', '--event-handlers', 'console_direct+',
        '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DUSE_GPU=OFF',
        '-DBUILD_TESTING=' + ('ON' if args.with_tests else 'OFF'),
    ]
    import shlex
    shell_command = 'source /opt/ros/humble/setup.bash && exec ' + shlex.join(command)
    environment = os.environ.copy()
    environment.update({
        'CMAKE_BUILD_PARALLEL_LEVEL': '1', 'MAKEFLAGS': '-j1 -l1',
        'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
        'MKL_NUM_THREADS': '1', 'CTEST_PARALLEL_LEVEL': '1',
    })
    # No other workspace overlay is sourced; avoid linking existing robot builds.
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                'LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION',
                'ROS_PYTHON_VERSION'):
        environment.pop(key, None)
    state = {'workspace': str(workspace), 'cpu': args.cpu, 'command': command,
             'started': datetime.datetime.now().astimezone().isoformat()}
    (output / 'status.json').write_text(json.dumps(state, indent=2))
    print('Build logs:', output, flush=True)

    def prepare_child():
        os.sched_setaffinity(0, {args.cpu})
        os.nice(10)

    process = subprocess.Popen(
        ['bash', '-c', shell_command], cwd=workspace, env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True, preexec_fn=prepare_child,
    )
    state['process_group'] = process.pid
    (output / 'status.json').write_text(json.dumps(state, indent=2))
    samples = []

    def copy_output():
        with (output / 'build.log').open('w') as logfile:
            for line in process.stdout:
                logfile.write(line)
                logfile.flush()
                print(line, end='', flush=True)

    reader = threading.Thread(target=copy_output, daemon=True)
    reader.start()
    exit_code = 130
    def interrupt_build(signum, frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt_build)
    try:
        with (output / 'memory.jsonl').open('w') as logfile:
            while True:
                sample = memory_sample()
                samples.append(sample)
                logfile.write(json.dumps(sample) + '\n')
                logfile.flush()
                if len(samples) % 6 == 1:
                    print('[memory] available=%.2f GiB swap_used=%.2f GiB %s' % (
                        sample['MemAvailable_KiB'] / 1048576,
                        (sample['SwapTotal_KiB'] - sample['SwapFree_KiB']) / 1048576,
                        sample['memory_pressure'].replace('\n', '; ')), flush=True)
                if process.poll() is not None:
                    exit_code = process.returncode
                    break
                time.sleep(5)
    except KeyboardInterrupt:
        print('Interrupted: stopping this build process group.', flush=True)
    finally:
        stop_group(process.pid)
        process.wait()
        reader.join(timeout=5)
        state.update({
            'finished': datetime.datetime.now().astimezone().isoformat(),
            'exit_code': exit_code,
            'remaining_build_processes': group_alive(process.pid),
            'min_MemAvailable_KiB': min(s['MemAvailable_KiB'] for s in samples) if samples else None,
            'max_swap_used_KiB': max(s['SwapTotal_KiB'] - s['SwapFree_KiB'] for s in samples) if samples else None,
        })
        (output / 'status.json').write_text(json.dumps(state, indent=2))
        print(json.dumps(state, indent=2), flush=True)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
