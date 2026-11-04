"""Start one Betaflight App per Betaflight SITL, each pre-set to connect to its drone over TCP.

Use as:
    python3 betaflight_app.py tcp://127.0.0.1:5761 [tcp://127.0.0.1:5771 ...]   (in drone order, drone k: port 5761 + 10 * (k - 1))
"""
import json
import os
import re
import sqlite3
import subprocess
import sys
import time

# The App keeps its settings in WebKit's localStorage (SQLite, values as UTF-16LE JSON {"key": value}), here one XDG_DATA_HOME per App
# Expert Mode shows the manual connection, firstRun skips the welcome dialog, portOverride is the address
# The App serves from tauri://localhost, which it treats as a development URL:
# automaticDevOptions=false keeps it from re-enabling Virtual Mode, so Connect picks the manual connection
urls = sys.argv[1:] or ['tcp://127.0.0.1:5761']
apps = {} # PID: (drone ID, URL)
for drone_id, url in enumerate(urls, start=1):
    data_home = os.path.expanduser(f'~/.local/share/betaflight_app_{drone_id}')
    storage_dir = os.path.join(data_home, 'com.betaflight.app', 'localstorage')
    os.makedirs(storage_dir, exist_ok=True)
    settings = {
        'firstRun': True,
        'expertMode': True,
        'automaticDevOptions': False,
        'showVirtualMode': False,
        'showManualMode': True,
        'portOverride': url
    }
    db = sqlite3.connect(os.path.join(storage_dir, 'tauri_localhost_0.localstorage'))
    db.execute('CREATE TABLE IF NOT EXISTS ItemTable (key TEXT UNIQUE ON CONFLICT REPLACE, value BLOB NOT NULL ON CONFLICT FAIL)')
    for key, value in settings.items():
        value_json = json.dumps({key: value}, separators=(',', ':')) # {"key":value}, as the App stores it
        db.execute('INSERT INTO ItemTable VALUES (?, ?)', (key, value_json.encode('utf-16-le')))
    db.commit()
    db.close()
    env = dict(os.environ, XDG_DATA_HOME=data_home)
    env.setdefault('WEBKIT_DISABLE_DMABUF_RENDERER', '1') # Avoid blank WebKitGTK windows
    app = subprocess.Popen(['betaflight-app'], env=env)
    apps[app.pid] = (drone_id, url)
    print(f'Betaflight App {drone_id}: Connect goes to {url}')

# Each window is renamed "Betaflight App: Drone k" and placed with wmctrl bottom-right of the screen
xrandr = subprocess.run(['xrandr'], capture_output=True, text=True).stdout
resolution = re.search(r'(\d+)x(\d+)', xrandr) # The first one listed
screen_w, screen_h = int(resolution.group(1)), int(resolution.group(2))
win_w = max(1024, 800 * screen_h // 1080) # 800x480 on Full HD, not below the App's minimum
win_h = max(550, 480 * screen_h // 1080)
pending = dict(apps) # Windows not placed yet
deadline = time.monotonic() + 60
while pending and time.monotonic() < deadline: # Title and place each window when it appears
    time.sleep(1)
    windows = subprocess.run(['wmctrl', '-lp'], capture_output=True, text=True).stdout
    for line in windows.splitlines(): # Window ID, desktop, PID, host, title
        window, _, pid = line.split()[:3]
        if int(pid) in pending:
            drone_id, url = pending.pop(int(pid))
            x = screen_w - win_w - 40 * (len(apps) - drone_id) # Where QGC goes, the other drones 40 px further left each
            subprocess.run(['wmctrl', '-i', '-r', window, '-N', f'Betaflight App: Drone {drone_id} ({url})'])
            subprocess.run(['wmctrl', '-i', '-r', window, '-e', f'0,{x},4096,{win_w},{win_h}']) # y = 4096 is clamped to the bottom

for _ in apps: # Until every App is closed
    os.wait()
