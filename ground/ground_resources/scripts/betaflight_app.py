"""Start one Betaflight App (instead of QGC) per Betaflight SITL, each pre-set to connect to its drone over TCP.

Usage:
    python3 betaflight_app.py tcp://127.0.0.1:5761 [tcp://127.0.0.1:5771 ...]   (in drone order, drone k: port 5761 + 10 * (k - 1))

- The App keeps its settings in WebKit's localStorage (SQLite, values as UTF-16LE JSON {"key": value}), here one XDG_DATA_HOME per App.
- Expert Mode shows the manual connection, firstRun skips the welcome dialog, portOverride is the address.
- The App serves from tauri://localhost, which it treats as a development URL: automaticDevOptions=false keeps it from re-enabling Virtual Mode, so Connect picks the manual connection.
- Each window is renamed "Betaflight App: Drone k" and placed where QGC goes (bottom-right, cascaded) with wmctrl.
"""
import os
import re
import sqlite3
import subprocess
import sys
import time

urls = sys.argv[1:] or ['tcp://127.0.0.1:5761']
apps = {}
for drone_id, url in enumerate(urls, start=1):
    data_home = os.path.expanduser(f'~/.local/share/betaflight_app_{drone_id}')
    storage_dir = os.path.join(data_home, 'com.betaflight.app', 'localstorage')
    os.makedirs(storage_dir, exist_ok=True)
    db = sqlite3.connect(os.path.join(storage_dir, 'tauri_localhost_0.localstorage'))
    db.execute('CREATE TABLE IF NOT EXISTS ItemTable (key TEXT UNIQUE ON CONFLICT REPLACE, value BLOB NOT NULL ON CONFLICT FAIL)')
    settings = (('firstRun', 'true'), ('expertMode', 'true'), ('automaticDevOptions', 'false'), ('showVirtualMode', 'false'),
                ('showManualMode', 'true'), ('portOverride', f'"{url}"'))
    for key, value in settings:
        db.execute('INSERT INTO ItemTable VALUES (?, ?)', (key, f'{{"{key}":{value}}}'.encode('utf-16-le')))
    db.commit()
    db.close()
    env = dict(os.environ, XDG_DATA_HOME=data_home)
    env.setdefault('WEBKIT_DISABLE_DMABUF_RENDERER', '1') # Avoid blank WebKitGTK windows on some GPU/container setups
    apps[subprocess.Popen(['betaflight-app'], env=env).pid] = (drone_id, url)
    print(f'Betaflight App {drone_id}: Connect goes to {url}')

screen_w, screen_h = map(int, re.search(r'(\d+)x(\d+)', subprocess.run(['xrandr'], capture_output=True, text=True).stdout).groups())
win_w, win_h = max(1024, 800 * screen_h // 1080), max(550, 480 * screen_h // 1080) # QGC's size (800x480 on Full HD), not below the App's minimum
pending, windows, deadline = dict(apps), {}, time.time() + 60 # Title and place each window when it appears
while pending and time.time() < deadline:
    time.sleep(1)
    for line in subprocess.run(['wmctrl', '-lp'], capture_output=True, text=True).stdout.splitlines(): # id, desktop, PID, host, title
        window, _, pid = line.split()[:3]
        if int(pid) in pending:
            drone_id, url = pending.pop(int(pid))
            windows[int(pid)] = window
            subprocess.run(['wmctrl', '-i', '-r', window, '-N', f'Betaflight App: Drone {drone_id} ({url})'])
            subprocess.run(['wmctrl', '-i', '-r', window, '-e', f'0,{screen_w - win_w - 40 * (len(apps) - drone_id)},4096,{win_w},{win_h}']) # Where QGC goes (y=4096 is clamped to the bottom), the other drones 40 px further left each

for _ in apps:
    os.wait()
