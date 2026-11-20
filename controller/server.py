#!/usr/bin/env python3
"""Camera-only HTTP service. No vehicle controller, GPIO, audio or write API."""
import argparse
import json
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
FILES = {
    '/': ('index.html', 'text/html; charset=utf-8'),
    '/index.html': ('index.html', 'text/html; charset=utf-8'),
    '/controller': ('index.html', 'text/html; charset=utf-8'),
    '/setting': ('index.html', 'text/html; charset=utf-8'),
    '/login': ('index.html', 'text/html; charset=utf-8'),
    '/logo-256.png': ('assets/logo-256.png', 'image/png'),
    '/assets/official.css': ('assets/official.css', 'text/css; charset=utf-8'),
    '/assets/viewer.css': ('assets/viewer.css', 'text/css; charset=utf-8'),
    '/assets/viewer.js': ('assets/viewer.js', 'text/javascript; charset=utf-8'),
    '/assets/viewer-config.js': ('assets/viewer-config.js', 'text/javascript; charset=utf-8'),
    '/assets/runtime.js': ('assets/runtime.js', 'text/javascript; charset=utf-8'),
    '/assets/vendor.js': ('assets/vendor.js', 'text/javascript; charset=utf-8'),
    '/assets/official-ui.js': ('assets/official-ui.js', 'text/javascript; charset=utf-8'),
    '/assets/logo-256.png': ('assets/logo-256.png', 'image/png'),
}


class Camera:
    def __init__(self, device):
        self.device = device
        self.condition = threading.Condition()
        self.clients = 0
        self.frame = None
        self.sequence = 0
        self.frame_time = 0
        self.error = ''
        self.worker = None
        self.process = None
        self.stopping = False

    def subscribe(self):
        with self.condition:
            self.clients += 1
            if not self.worker or not self.worker.is_alive():
                self.frame = None
                self.error = ''
                self.worker = threading.Thread(target=self.capture, daemon=True)
                self.worker.start()

    def unsubscribe(self):
        with self.condition:
            self.clients = max(0, self.clients - 1)
            self.condition.notify_all()

    def capture(self):
        proc = None
        try:
            if not Path(self.device).exists():
                raise RuntimeError('未找到 H65 摄像头，请检查 USB 连接。')
            # Do not open a device already held by the race or another viewer.
            if shutil.which('fuser'):
                busy = subprocess.run(['fuser', self.device], stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL, timeout=3)
                if busy.returncode == 0:
                    raise RuntimeError('摄像头正在被其他程序使用；只读页面不会停止该程序。')
            proc = subprocess.Popen([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
                '-f', 'v4l2', '-input_format', 'mjpeg', '-video_size', '640x480',
                '-framerate', '15', '-i', self.device, '-an', '-c:v', 'copy',
                '-f', 'image2pipe', 'pipe:1'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
            with self.condition:
                self.process = proc
            errors = []

            def drain_errors():
                for line in iter(proc.stderr.readline, b''):
                    errors.append(line.decode('utf-8', 'replace').strip())
                    del errors[:-5]

            threading.Thread(target=drain_errors, daemon=True).start()
            pending = bytearray()
            idle_since = None
            while not self.stopping:
                with self.condition:
                    if self.clients == 0:
                        idle_since = idle_since or time.monotonic()
                        if time.monotonic() - idle_since > 1:
                            break
                    else:
                        idle_since = None
                chunk = proc.stdout.read(65536)
                if not chunk:
                    raise RuntimeError('摄像头采集已中断。' + (' ' + errors[-1] if errors else ''))
                pending.extend(chunk)
                while True:
                    start = pending.find(b'\xff\xd8')
                    end = pending.find(b'\xff\xd9', max(0, start + 2))
                    if start < 0 or end < 0:
                        break
                    jpeg = bytes(pending[start:end + 2])
                    del pending[:end + 2]
                    with self.condition:
                        self.frame = jpeg
                        self.sequence += 1
                        self.frame_time = time.monotonic()
                        self.error = ''
                        self.condition.notify_all()
                if len(pending) > 4 * 1024 * 1024:
                    raise RuntimeError('摄像头没有返回有效的 MJPEG 画面。')
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
                self.frame = None
                self.condition.notify_all()
        finally:
            if proc:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                proc.stdout.close()
                proc.stderr.close()
            with self.condition:
                self.process = None
                self.condition.notify_all()

    def next_frame(self, previous):
        deadline = time.monotonic() + 8
        with self.condition:
            while not self.error and not self.stopping:
                if self.frame is not None and self.sequence != previous:
                    return self.sequence, self.frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(remaining)
            return previous, None

    def status(self):
        with self.condition:
            return {'read_only': True, 'clients': self.clients, 'frame': self.sequence,
                    'frame_age': round(time.monotonic() - self.frame_time, 2) if self.frame_time else None,
                    'error': self.error}

    def stop(self):
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
            if self.process:
                self.process.terminate()
        if self.worker:
            self.worker.join(timeout=5)


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, content_type, body, head=False):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        if not head:
            self.wfile.write(body)

    def do_GET(self):
        self.serve()

    def do_HEAD(self):
        self.serve(head=True)

    def serve(self, head=False):
        path = urlsplit(self.path).path
        if path.startswith('/setting/') or path.startswith('/controller/'):
            path = '/'
        if path in FILES:
            file_name, mime = FILES[path]
            self.reply(200, mime, (ROOT / file_name).read_bytes(), head)
        elif path == '/status':
            self.reply(200, 'application/json', json.dumps(self.server.camera.status()).encode(), head)
        elif path == '/stream.mjpg' and not head:
            self.stream()
        else:
            self.reply(404, 'text/plain', b'Camera-only service: no such endpoint.', head)

    def stream(self):
        camera = self.server.camera
        camera.subscribe()
        try:
            sequence, frame = camera.next_frame(-1)
            if frame is None:
                self.reply(503, 'text/plain; charset=utf-8',
                           (camera.status()['error'] or '摄像头未返回画面。').encode('utf-8'))
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.connection.settimeout(5)
            while frame is not None:
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                 str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                self.wfile.flush()
                sequence, frame = camera.next_frame(sequence)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass
        finally:
            camera.unsubscribe()

    def reject_write(self):
        self.reply(405, 'text/plain', b'Read-only camera service. Writes are disabled.')

    do_POST = do_PUT = do_PATCH = do_DELETE = reject_write

    def log_message(self, fmt, *args):
        # Status polling does not need one log line every two seconds.
        if urlsplit(self.path).path != '/status':
            super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8086)
    parser.add_argument('--camera', default='/dev/v4l/by-id/usb-H65_USB_CAMERA_H65_USB_CAMERA-video-index0')
    args = parser.parse_args()
    if not shutil.which('ffmpeg'):
        parser.error('ffmpeg is required (already installed on the competition Pi).')
    camera = Camera(args.camera)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    server.daemon_threads = True
    server.camera = camera

    def shutdown(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print('Camera-only viewer: http://192.168.137.139:{}/'.format(args.port), flush=True)
    print('Camera opens only while a viewer is connected. Ctrl+C to stop.', flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        camera.stop()
        server.server_close()


if __name__ == '__main__':
    main()
