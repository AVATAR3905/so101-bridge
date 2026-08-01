"""
HTTP server — serves:
  /             — GUI (index.html) — open http://localhost:8766 in browser
  /cam/<dev>    — MJPEG camera stream
  /snapshot/<dev> — single JPEG snapshot
  /stats        — camera stats JSON
"""

import asyncio
import base64
import json
import logging
from pathlib import Path

import cv2
from aiohttp import web

log = logging.getLogger("mjpeg")

_camera_manager = None
_hand_frame_provider = None  # callable returning BGR np.ndarray or None
_static_dir = Path(__file__).parent / "static"


def set_camera_manager(cm):
    global _camera_manager
    _camera_manager = cm


def set_hand_frame_provider(fn):
    global _hand_frame_provider
    _hand_frame_provider = fn


async def _index_handler(request: web.Request) -> web.Response:
    """Serve the GUI HTML over HTTP so WebSocket works correctly."""
    index = _static_dir / "index.html"
    if not index.exists():
        return web.Response(status=404, text="index.html not found in static/")
    return web.Response(
        body=index.read_bytes(),
        content_type="text/html",
        headers={"Cache-Control": "no-cache"},
    )


async def _stream_handler(request: web.Request) -> web.StreamResponse:
    dev_id = int(request.match_info["dev"])
    response = web.StreamResponse(
        status=200,
        headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=--jpgboundary",
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "Access-Control-Allow-Origin": "*",
        }
    )
    await response.prepare(request)
    try:
        while True:
            if _camera_manager is None:
                await asyncio.sleep(0.1)
                continue
            jpeg_b64 = _camera_manager.get_jpeg_b64(dev_id, quality=70)
            if jpeg_b64:
                data = base64.b64decode(jpeg_b64)
                header = (
                    b"--jpgboundary\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n"
                )
                await response.write(header + data + b"\r\n")
            await asyncio.sleep(1 / 30)
    except (asyncio.CancelledError, ConnectionResetError):
        pass
    return response


async def _snapshot_handler(request: web.Request) -> web.Response:
    dev_id = int(request.match_info["dev"])
    if _camera_manager is None:
        return web.Response(status=503, text="Camera manager not ready")
    jpeg_b64 = _camera_manager.get_jpeg_b64(dev_id, quality=90)
    if not jpeg_b64:
        return web.Response(status=404, text="No frame available")
    return web.Response(body=base64.b64decode(jpeg_b64), content_type="image/jpeg")


async def _stats_handler(request: web.Request) -> web.Response:
    if _camera_manager is None:
        return web.Response(status=503, text="Not ready")
    return web.Response(
        text=json.dumps(_camera_manager.get_stats()),
        content_type="application/json",
    )


async def _episode_frame_handler(request: web.Request) -> web.Response:
    name = request.match_info["name"]
    cam  = request.match_info["cam"]
    idx  = request.match_info["idx"]
    dataset_dir = request.app.get("dataset_dir", str(Path.home() / "datasets" / "so101"))
    frame_path = Path(dataset_dir).expanduser() / name / "frames" / f"cam_{cam}_{int(idx):06d}.jpg"
    if not frame_path.exists():
        return web.Response(status=404, text="Frame not found")
    return web.Response(body=frame_path.read_bytes(), content_type="image/jpeg",
                        headers={"Cache-Control": "no-cache"})


async def _hand_stream_handler(request: web.Request) -> web.StreamResponse:
    response = web.StreamResponse(
        status=200,
        headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=--jpgboundary",
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "Access-Control-Allow-Origin": "*",
        }
    )
    await response.prepare(request)
    try:
        while True:
            frame = _hand_frame_provider() if _hand_frame_provider else None
            if frame is not None:
                ret, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    data = buf.tobytes()
                    header = (
                        b"--jpgboundary\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n"
                    )
                    await response.write(header + data + b"\r\n")
            await asyncio.sleep(1 / 25)
    except (asyncio.CancelledError, ConnectionResetError):
        pass
    return response


def make_app(dataset_dir: str = "") -> web.Application:
    app = web.Application()
    app["dataset_dir"] = dataset_dir
    app.router.add_get("/",              _index_handler)
    app.router.add_get("/index.html",    _index_handler)
    app.router.add_get("/cam/{dev}",     _stream_handler)
    app.router.add_get("/snapshot/{dev}", _snapshot_handler)
    app.router.add_get("/stats",         _stats_handler)
    app.router.add_get("/episode-frame/{name}/{cam}/{idx}", _episode_frame_handler)
    app.router.add_get("/cam/hand", _hand_stream_handler)
    return app


async def start_mjpeg_server(host: str = "0.0.0.0", port: int = 8766, dataset_dir: str = ""):
    app = make_app(dataset_dir=dataset_dir)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    log.info("HTTP server  on http://%s:%d  (open this in your browser)", host, port)
    log.info("MJPEG cams   at http://%s:%d/cam/0  and /cam/2", host, port)
    return runner
