#!/usr/bin/env python3
"""Webcam MJPEG HTTP server — stream Windows webcam into Docker over HTTP.

Usage (PowerShell or CMD on Windows — NOT from Kali/WSL):
    python scripts/webcam_stream.py
    python scripts/webcam_stream.py --device 0 --port 8554

Then in Docker:
    /home/ubuntu/scripts/launch_vision.sh
"""

import argparse
import socket
import sys
import threading
import time

import cv2


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--device',  type=int, default=0)
    p.add_argument('--port',    type=int, default=8554)
    p.add_argument('--width',   type=int, default=640)
    p.add_argument('--height',  type=int, default=480)
    p.add_argument('--fps',     type=int, default=30)
    p.add_argument('--quality', type=int, default=85)
    return p.parse_args()


# ── shared latest-frame ────────────────────────────────────────────────────────
_lock   = threading.Lock()
_frame  = None          # bytes: latest JPEG
_ready  = threading.Event()


def capture_loop(device, width, height, fps, quality):
    global _frame
    cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS,          fps)
    if not cap.isOpened():
        print(f'[webcam_stream] ERROR: cannot open device {device}', flush=True)
        sys.exit(1)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'[webcam_stream] Camera opened: device={device} {w}x{h}', flush=True)
    enc = [cv2.IMWRITE_JPEG_QUALITY, quality]
    while True:
        ret, img = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
        _, jpg = cv2.imencode('.jpg', img, enc)
        with _lock:
            _frame = jpg.tobytes()
        _ready.set()


def handle_client(conn: socket.socket, addr):
    try:
        # Drain the HTTP request (we don't care about path/headers)
        conn.settimeout(2.0)
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk or b'\r\n\r\n' in chunk:
                    break
        except socket.timeout:
            pass
        conn.settimeout(None)

        print(f'[webcam_stream] Client connected: {addr[0]}', flush=True)

        # Send MJPEG response headers
        headers = (
            b'HTTP/1.1 200 OK\r\n'
            b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n'
            b'Cache-Control: no-cache\r\n'
            b'Connection: close\r\n'
            b'\r\n'
        )
        conn.sendall(headers)

        while True:
            _ready.wait(timeout=2.0)
            with _lock:
                data = _frame
            if data is None:
                continue
            part = (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                b'Content-Length: ' + str(len(data)).encode() + b'\r\n'
                b'\r\n'
                + data +
                b'\r\n'
            )
            conn.sendall(part)

    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        print(f'[webcam_stream] Client disconnected: {addr[0]}', flush=True)
        conn.close()


def main():
    args = parse_args()

    t = threading.Thread(
        target=capture_loop,
        args=(args.device, args.width, args.height, args.fps, args.quality),
        daemon=True,
    )
    t.start()

    print('[webcam_stream] Waiting for first frame...', flush=True)
    _ready.wait(timeout=10.0)
    if _frame is None:
        print('[webcam_stream] ERROR: no frames from camera, exiting.', flush=True)
        sys.exit(1)

    srv = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # IPv6 wildcard with IPV6_V6ONLY=0 accepts both IPv4 and IPv6
    srv.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
    srv.bind(('::', args.port))
    srv.listen(5)

    print(f'[webcam_stream] Serving on http://0.0.0.0:{args.port}/ (IPv4+IPv6)', flush=True)
    print(f'[webcam_stream] Docker: http://host.docker.internal:{args.port}/', flush=True)
    print('[webcam_stream] Press Ctrl+C to stop.', flush=True)

    while True:
        try:
            conn, addr = srv.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
        except KeyboardInterrupt:
            break

    srv.close()


if __name__ == '__main__':
    main()
