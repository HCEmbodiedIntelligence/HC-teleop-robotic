"""Desktop entry point. The browser owns the operator account and robot session."""

import argparse
import ctypes
from ctypes import wintypes
import threading
import tkinter as tk
from tkinter import messagebox, ttk
import webbrowser

from windows_client.bridge import BrowserBridge, DEFAULT_SITE, Receiver, site_origin

TITLE = "HC 遥操作 · 设备连接"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", default=DEFAULT_SITE)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    root = tk.Tk()
    root.withdraw()
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.CreateMutexW.argtypes = [ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR]
    kernel.CreateMutexW.restype = wintypes.HANDLE
    mutex = kernel.CreateMutexW(None, False, "Local\\HC.Teleop.Workstation.v1")
    if ctypes.get_last_error() == 183:
        user = ctypes.windll.user32
        user.FindWindowW.restype = wintypes.HWND
        user.FindWindowW.argtypes = [wintypes.LPCWSTR, wintypes.LPCWSTR]
        hwnd = user.FindWindowW(None, TITLE)
        if hwnd:
            user.ShowWindow(wintypes.HWND(hwnd), 9)
            user.SetForegroundWindow(wintypes.HWND(hwnd))
        root.destroy()
        return
    receiver = Receiver()
    try:
        site_origin(args.site)
        bridge = BrowserBridge(receiver, args.site)
    except (OSError, ValueError) as exc:
        messagebox.showerror("HC 遥操作启动失败", str(exc))
        root.destroy()
        return
    receiver.start()
    threading.Thread(target=bridge.serve_forever, daemon=True).start()
    root.title(TITLE)
    root.minsize(505, 362)
    root.configure(bg="#f4f6fa")
    outer = ttk.Frame(root, padding=(26, 20))
    outer.pack(fill="both", expand=True)
    ttk.Label(outer, text="设备连接", font=("Microsoft YaHei UI", 18, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 14))
    outer.columnconfigure(1, weight=1)
    values = {}
    for row, label in enumerate(("PICO 数据源", "头显跟踪", "左手柄", "右手柄", "网页连接"), 1):
        ttk.Label(outer, text=label).grid(row=row, column=0, sticky="w", padx=(0, 20), pady=5)
        values[label] = ttk.Label(outer, text="等待检测", wraplength=340)
        values[label].grid(row=row, column=1, sticky="w")
    note = tk.StringVar(value="请在 PICO 中打开遥操作应用，头显与电脑连接同一局域网。")
    ttk.Label(outer, textvariable=note, wraplength=460).grid(row=6, column=0, columnspan=2, sticky="w", pady=(14, 8))
    buttons = ttk.Frame(outer)
    buttons.grid(row=7, column=0, columnspan=2, sticky="w")

    def open_page():
        ticket = bridge.issue()
        if not webbrowser.open(args.site + "#hc-workstation=" + ticket):
            note.set("浏览器未打开。请打开平台遥操作页，点击「复制配对码」并粘贴。")

    def copy_code():
        root.clipboard_clear()
        root.clipboard_append(bridge.issue())
        note.set("配对码已复制，3 分钟内在遥操作网页的「连接本机」中粘贴。")

    ttk.Button(buttons, text="打开遥操作网页", command=open_page).pack(side="left", padx=(0, 8))
    ttk.Button(buttons, text="复制配对码", command=copy_code).pack(side="left")

    def update():
        view = receiver.snapshot()
        frame = view["frame"]
        values["PICO 数据源"]["text"] = view["error"] or view["peer"] or "等待 PICO 输入"
        values["头显跟踪"]["text"] = "已跟踪" if frame and frame["tracking"]["head"] else "等待有效位姿"
        for hand, label in (("left", "左手柄"), ("right", "右手柄")):
            value = "等待有效位姿"
            if frame and frame["tracking"][hand]:
                trigger = frame["inputs"][hand]["trigger"]
                value = f"已跟踪    扳机 {trigger:.2f}" if frame["protocol_version"] == 2 else "已跟踪    v1 仅位姿"
            values[label]["text"] = value
        values["网页连接"]["text"] = "已连接" if bridge.active_stream else "等待网页连接"
        root.after(150, update)

    root.deiconify()
    update()
    if not args.no_browser:
        root.after(400, open_page)
    try:
        root.mainloop()
    finally:
        bridge.stop()
        receiver.close()
        kernel.CloseHandle(wintypes.HANDLE(mutex))


if __name__ == "__main__":
    main()
