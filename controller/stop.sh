#!/bin/bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec python3 -B - "$PWD/server.py" <<'PY'
import os
from pathlib import Path
import re
import signal
import sys
import time

target = Path(sys.argv[1]).resolve()


def identify(pid):
    """Match this exact Python script, including relative-path launches."""
    proc = Path('/proc') / str(pid)
    try:
        executable = (proc / 'exe').resolve(strict=True).name
        if not re.fullmatch(r'python(?:\d+(?:\.\d+)*)?', executable):
            return None
        args = (proc / 'cmdline').read_bytes().split(b'\0')
        script = None
        for arg in args[1:]:
            value = os.fsdecode(arg)
            if value in ('-c', '-m'):
                return None
            if value and not value.startswith('-'):
                script = Path(value)
                break
        if script is None:
            return None
        if not script.is_absolute():
            script = (proc / 'cwd').resolve(strict=True) / script
        if script.resolve() != target:
            return None
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else fields[19]  # process start time
    except (OSError, ValueError, IndexError):
        return None


running = {}
for entry in Path('/proc').iterdir():
    if entry.name.isdigit():
        pid = int(entry.name)
        started = identify(pid)
        if started is not None:
            running[pid] = started

if not running:
    print('只读视频服务未运行，无需停止。')
    sys.exit(0)

for pid, started in running.items():
    if identify(pid) != started:
        continue
    try:
        os.kill(pid, signal.SIGTERM)  # server.py releases its own camera process
    except ProcessLookupError:
        pass
    except PermissionError:
        sys.exit('没有权限停止该视频服务，请使用启动它的用户运行 stop.sh。')

deadline = time.monotonic() + 10
while time.monotonic() < deadline:
    pending = [pid for pid, started in running.items() if identify(pid) == started]
    if not pending:
        print('只读视频服务已停止，摄像头采集已关闭。')
        sys.exit(0)
    time.sleep(0.1)

sys.exit('已发送停止请求，但服务尚未退出；请检查 PID: ' + ', '.join(map(str, pending)))
PY
