"""Release smoke test on Windows. Requires Playwright and Microsoft Edge on the build PC."""

import argparse
import ctypes
import http.client
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import urlsplit

from middleware.core.protocol import PACKET_FORMAT
from windows_client.bridge import PORT


SITE = "https://hc-workstation-test.invalid/teleoperation"


def test_package(exe, web_bundle=None):
    from playwright.sync_api import sync_playwright

    # Capture the normal default-browser invocation in an isolated test profile.
    with tempfile.TemporaryDirectory() as temp:
        captured = Path(temp) / "launch-url"
        capture_program = Path(temp) / "capture.py"
        capture_program.write_text("import sys\nfrom pathlib import Path\nPath(sys.argv[1]).write_text(sys.argv[2])\n")
        browser_command = ' '.join('"' + p.as_posix() + '"' for p in (Path(sys.executable), capture_program, captured))
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        process = subprocess.Popen([str(exe), "--site", SITE],
                                   env={**os.environ, "BROWSER": browser_command + " %s"}, startupinfo=startup)
        done = threading.Event()
        try:
            deadline = time.monotonic() + 24
            while not captured.exists() and process.poll() is None and time.monotonic() < deadline:
                time.sleep(.1)
            assert captured.exists(), "EXE did not open the workstation page"
            url = captured.read_text()
            assert url.startswith(SITE + "#hc-workstation=")
            ticket = urlsplit(url).fragment.split("=", 1)[1]
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                udp.settimeout(3)
                udp.sendto(b"PICO_DISCOVER_V1", ("127.0.0.1", 5006))
                assert udp.recv(256) == b"PICO_RECEIVER_V1|5005"
            def publish():
                seq = 0
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                    pose = [0, 1, 0, 0, 0, 0, 1] * 3
                    while not done.wait(.015):
                        seq += 1
                        data = struct.pack(PACKET_FORMAT, b"PICO", 2, seq, time.monotonic(), 7, *pose,
                                           0, 0, 0, .25, 0., 0., 0., 0., 0.,
                                           0, 0, 0, .75, 0., 0., 0., 0., 0.)
                        udp.sendto(data, ("127.0.0.1", 5005))
            emitter = threading.Thread(target=publish, daemon=True)
            emitter.start()
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(channel="msedge", headless=True)
                context = browser.new_context()
                page = context.new_page()
                context.grant_permissions(["local-network-access"], origin=SITE.split("/teleoperation")[0])
                page.route(SITE, lambda route: route.fulfill(content_type="text/html", body="<title>Workstation QA</title>"))
                page.goto(url)
                if web_bundle:
                    page.add_script_tag(path=str(web_bundle))
                    script = Path(__file__).with_name("tests") / "webrtc-smoke.js"
                    page.evaluate(script.read_text(encoding="utf-8"))
                    page.wait_for_function("window.ready && window.videoReady", timeout=12000)
                    conflict = page.evaluate("""async () => {
                      try {
                        await HCWeb.readWorkstation("release-test", AbortSignal.timeout(1500), () => {});
                      } catch (error) { return error.message; }
                    }""")
                    assert "另一标签页" in conflict, "a second tab could interrupt the active workstation stream"
                    assert page.evaluate("window.received.length") == 0
                    page.evaluate("window.actions.enable()")
                    page.wait_for_function("window.received.at(-1)?.controllers[1]?.buttons[0] === 0.75", timeout=5000)
                else:
                    page.evaluate("""async () => {
                      const base = 'http://127.0.0.1:47653';
                      const ticket = location.hash.split('=')[1];
                      const pair = await fetch(base+'/pair',{method:'POST',headers:{Authorization:'Bearer '+ticket}});
                      if(!pair.ok) throw new Error('pair: '+pair.status);
                      const grant = (await pair.json()).grant;
                      const stream = await fetch(base+'/stream',{headers:{Authorization:'Bearer '+grant}});
                      const reader = stream.body.pipeThrough(new TextDecoderStream()).getReader();
                      window.qa = {};
                      void (async () => {
                        let part = '';
                        for (;;) {
                          const {value,done} = await reader.read(); if(done) return;
                          part += value;
                          const rows = part.split('\\n'); part = rows.pop();
                          for(const row of rows) window.qa = JSON.parse(row);
                        }
                      })();
                    }""")
                page.wait_for_function("window.qa.frame?.inputs.right.trigger === 0.75 && window.qa.frame?.tracking.left", timeout=5000)
                done.set()
                page.wait_for_function("window.qa.frame === null", timeout=2500)
                if web_bundle:
                    page.wait_for_function("window.stopped && !window.enabled", timeout=2500)
                    print("PASS: production browser reader and WebRTC sender, video preview, explicit enable/ack, synthetic robot input and stop")
                browser.close()
            conn = http.client.HTTPConnection("127.0.0.1", PORT, timeout=3)
            conn.request("POST", "/pair", headers={"Origin": SITE.rsplit("/", 1)[0], "Authorization": "Bearer " + ticket})
            assert conn.getresponse().status == 401, "pairing ticket was reusable"
            conn.close()
            print("PASS: installed EXE, browser launch, UDP discovery, Edge pairing, v2 sample, stale input, single-use ticket")
        finally:
            done.set()
            # Close only the window launched by this smoke test.
            if captured.exists() and process.poll() is None:
                from ctypes import wintypes
                user = ctypes.windll.user32
                user.FindWindowW.argtypes = [wintypes.LPCWSTR, wintypes.LPCWSTR]
                user.FindWindowW.restype = wintypes.HWND
                hwnd = user.FindWindowW(None, "HC 遥操作 · 设备连接")
                if hwnd:
                    user.PostMessageW(wintypes.HWND(hwnd), 0x0010, 0, 0)
            if process.poll() is None:
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    process.terminate()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("exe", type=Path)
    parser.add_argument("--web-bundle", type=Path)
    args = parser.parse_args()
    test_package(args.exe.resolve(), args.web_bundle)
