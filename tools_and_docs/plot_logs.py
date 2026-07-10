"""
Plot flight data from the .ulg (PX4) and .BIN (ArduPilot) logs

Use as:
    python3 plot_logs.py /path/to/logs/folder
"""

import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pymap3d

def read_ulg(ulg_file):
    # Extract the lat, lon, alt trajectory and the saved home point from a PX4 .ulg log
    from pyulog import ULog
    ulog = ULog(ulg_file, ['vehicle_global_position', 'home_position'])
    data = ulog.get_dataset('vehicle_global_position').data
    # resets = data.get('lat_lon_reset_counter')
    # if resets is not None and resets[-1] != resets[0]:
    #     print(f'Warning: {int(resets[-1]) - int(resets[0])} EKF lat/lon reset(s) during {os.path.basename(ulg_file)}')
    try:
        home = ulog.get_dataset('home_position').data
        home = (home['lat'][0], home['lon'][0], home['alt'][0])
    except Exception:
        home = (data['lat'][0], data['lon'][0], data['alt'][0])  # Fallback: use first streamed sample
    return data['timestamp'], data['lat'], data['lon'], data['alt'], home

def read_bin(bin_file):
    # Extract the lat, lon, alt trajectory from an ArduPilot .BIN log
    from pymavlink import mavutil
    connection = mavutil.mavlink_connection(bin_file)
    t, lat, lon, alt = [], [], [], []
    while (msg := connection.recv_match(type='POS')) is not None:
        t.append(msg.TimeUS)
        lat.append(msg.Lat)
        lon.append(msg.Lng)
        alt.append(msg.Alt)
    if not lat:
        raise ValueError('No POS messages in log')
    t, lat, lon, alt = np.array(t), np.array(lat), np.array(lon), np.array(alt)
    return t, lat, lon, alt, (lat[0], lon[0], alt[0])  # ArduPilot sets home at arming, when POS logging also starts

if __name__ == '__main__':
    log_dir = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    log_files = sorted(glob.glob(os.path.join(log_dir, '*.ulg')) + glob.glob(os.path.join(log_dir, '*.BIN')))
    if not log_files:
        sys.exit(f'No .ulg or .BIN logs found in {log_dir}')
    fig = plt.figure(num='Flight Paths', figsize=(16, max(8, 3 * len(log_files))), layout='constrained')
    gs = fig.add_gridspec(2 * len(log_files), 2, width_ratios=[2, 1])
    ax = fig.add_subplot(gs[:, 0], projection='3d')
    origin = None  # Saved home of the first readable log, commong to all trajectories
    for k, log_file in enumerate(log_files):
        label = os.path.splitext(os.path.basename(log_file))[0]
        try:
            t_us, lat, lon, alt, home = read_ulg(log_file) if log_file.endswith('.ulg') else read_bin(log_file)
            if origin is None:
                origin = home
            east, north, up = pymap3d.geodetic2enu(lat, lon, alt, *origin)
            t = (t_us - t_us[0]) / 1e6
            hspeed = np.hypot(np.gradient(east, t), np.gradient(north, t))
            home_east, home_north, _ = pymap3d.geodetic2enu(*home, *origin)
            line, = ax.plot(east, north, up, alpha=0.6, label=label)
            ax.scatter(east[0], north[0], up[0], color=line.get_color(), marker='o')  # Marker on the first sample of this log
            ax_alt = fig.add_subplot(gs[2 * k, 1])
            ax_spd = fig.add_subplot(gs[2 * k + 1, 1], sharex=ax_alt)
            ax_alt.plot(t, up, color=line.get_color(), alpha=0.8)
            ax_alt.set_ylabel('Alt. [m]')
            ax_alt.set_title(label, fontsize=10)
            ax_alt.tick_params(labelbottom=False)
            ax_spd.plot(t, hspeed, color=line.get_color(), alpha=0.8)
            ax_spd.set_ylabel('X-Y Speed [m/s]')
            ax_spd.set_xlabel('Time [s]')
        except Exception as e:
            print(f'Skipping {label}: {e}')
    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.set_zlabel('Up [m]')
    ax.set_title(os.path.basename(os.path.normpath(log_dir)))
    ax.legend()
    try:
        ax.set_aspect('equal')
    except NotImplementedError:
        pass  # 3D equal aspect requires matplotlib >= 3.7
    plot_file = os.path.join(log_dir, 'trajectories.png')
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f'Saved: {plot_file}')
    plt.show()
