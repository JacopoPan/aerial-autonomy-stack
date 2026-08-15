"""
Plot flight data from the .ulg (PX4) and .bin/.BIN (ArduPilot) logs

Use as:
    python3 plot_logs.py /path/to/logs/folder
"""

import glob
import itertools
import os
import sys
import warnings

import matplotlib.pyplot as plt
import numpy as np
import pymap3d

GPS_UTC_EPOCH = 315964800 - 18 # Unix seconds at the GPS epoch (1980-01-06), less the leap seconds until 2026 https://en.wikipedia.org/wiki/Leap_second

def read_ulg(ulg_file):
    # Extract the lat, lon, alt trajectory and the saved home point from a PX4 .ulg log
    from pyulog import ULog
    ulog = ULog(ulg_file, ['vehicle_global_position', 'home_position', 'vehicle_local_position', 'sensor_gps'])
    data = ulog.get_dataset('vehicle_global_position').data
    resets = data.get('lat_lon_reset_counter')
    if resets is not None and resets[-1] != resets[0]:
        print(f'Warning: {int(resets[-1]) - int(resets[0])} EKF lat/lon reset(s) during {os.path.basename(ulg_file)}')
    try:
        home = ulog.get_dataset('home_position').data
        home = (home['lat'][0], home['lon'][0], home['alt'][0])
    except Exception:
        home = (data['lat'][0], data['lon'][0], data['alt'][0]) # Fallback: use first streamed sample
    vel = ulog.get_dataset('vehicle_local_position').data
    try:
        gps = ulog.get_dataset('sensor_gps').data
        i = int(np.flatnonzero(gps['time_utc_usec'])[0]) # First sample carrying a UTC fix
        utc_offset = int(gps['time_utc_usec'][i]) - int(gps['timestamp'][i]) - int(gps['timestamp_time_relative'][i])
    except Exception:
        utc_offset = 0 # SITL publishes no UTC, but its lockstep clock is already shared across instances
    return data['timestamp'].astype(np.int64) + utc_offset, data['lat'], data['lon'], data['alt'], home, vel['timestamp'].astype(np.int64) + utc_offset, vel['vx'], vel['vy']

def read_bin(bin_file):
    # Extract the lat, lon, alt trajectory from an ArduPilot .BIN log
    from pymavlink import mavutil
    connection = mavutil.mavlink_connection(bin_file)
    t, lat, lon, alt = [], [], [], []
    t_spd, vn, ve = [], [], []
    utc_offset = None
    while (msg := connection.recv_match(type=['POS', 'XKF1', 'GPS'])) is not None:
        if msg.get_type() == 'POS':
            t.append(msg.TimeUS)
            lat.append(msg.Lat)
            lon.append(msg.Lng)
            alt.append(msg.Alt)
        elif msg.get_type() == 'GPS':
            if utc_offset is None and msg.GWk > 0: # First message with a locked week number
                utc_offset = int((msg.GWk * 604800 + msg.GMS / 1e3 + GPS_UTC_EPOCH) * 1e6 - msg.TimeUS)
        elif msg.get_type() == 'XKF1' and msg.C == 0: # Velocity from the first EKF core
            t_spd.append(msg.TimeUS)
            vn.append(msg.VN)
            ve.append(msg.VE)
    if not lat:
        raise ValueError('No POS messages in log')
    t, lat, lon, alt = np.array(t), np.array(lat), np.array(lon), np.array(alt)
    t_spd, vn, ve = np.array(t_spd), np.array(vn), np.array(ve)
    if utc_offset is None:
        utc_offset = 0
        print(f'Warning: no GPS time in {os.path.basename(bin_file)}, its time axis is not aligned with the other logs')
    return t + utc_offset, lat, lon, alt, (lat[0], lon[0], alt[0]), t_spd + utc_offset, vn, ve # ArduPilot sets home at arming, when POS logging also starts

if __name__ == '__main__':
    log_dir = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    log_files = sorted(f for f in glob.glob(os.path.join(log_dir, '*')) if f.lower().endswith(('.ulg', '.bin')))
    if not log_files:
        sys.exit(f'No .ulg or .bin logs found in {log_dir}')
    fig = plt.figure(num='Flight Summary', figsize=(16, max(8, 3 * len(log_files))), layout='constrained')
    gs = fig.add_gridspec(2 * len(log_files), 2, width_ratios=[2, 1])
    gs_left = gs[:, 0].subgridspec(4, 1) # Split the left column so the inter-drone distance plot is a quarter of its height
    ax = fig.add_subplot(gs_left[:3, 0] if len(log_files) > 1 else gs_left[:, 0], projection='3d')
    origin = None # Saved home of the first readable log, common to all trajectories
    ax_time = None # First time axis, shared by every later panel
    tracks = [] # (label, time, ENU) per readable log, all on a shared origin and clock
    for k, log_file in enumerate(log_files):
        label = os.path.splitext(os.path.basename(log_file))[0]
        try:
            t_us, lat, lon, alt, home, t_spd_us, vn, ve = read_ulg(log_file) if log_file.lower().endswith('.ulg') else read_bin(log_file)
            if origin is None:
                origin = home
                t_zero_us = t_us[0] # Time of the first sample of the first log, common zero for every time axis
            east, north, up = pymap3d.geodetic2enu(lat, lon, alt, *origin)
            t = (t_us - t_zero_us) / 1e6
            t_spd = (t_spd_us - t_zero_us) / 1e6 # Same zero as t, to keep the time axes aligned
            if abs(t[0]) > 86400: # A day is a safe threshold
                print(f'Warning: {label} is not on the same clock as the reference log')
            tracks.append((label, t, np.array([east, north, up])))
            hspeed = np.hypot(vn, ve)
            line, = ax.plot(east, north, up, alpha=0.6, label=label)
            ax.scatter(east[0], north[0], up[0], color=line.get_color(), marker='o')  # Marker on the first sample of this log
            ax_alt = fig.add_subplot(gs[2 * k, 1], sharex=ax_time)
            if ax_time is None:
                ax_time = ax_alt
            ax_spd = fig.add_subplot(gs[2 * k + 1, 1], sharex=ax_time)
            ax_alt.plot(t, up, color=line.get_color(), alpha=0.8)
            ax_alt.set_ylabel('Up [m]')
            ax_alt.set_title(label, fontsize=10)
            ax_alt.tick_params(labelbottom=False)
            ax_spd.plot(t_spd, hspeed, color=line.get_color(), alpha=0.8)
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
        pass # 3D equal aspect requires matplotlib >= 3.7
    if len(tracks) > 1:
        ax_dist = fig.add_subplot(gs_left[3, 0], sharex=ax_time)
        for (label_a, t_a, enu_a), (label_b, t_b, enu_b) in itertools.combinations(tracks, 2):
            overlap = (t_a >= t_b[0]) & (t_a <= t_b[-1]) # Samples of a inside b's span, outside which np.interp would clamp to the end values
            if not overlap.any():
                print(f'Warning: {label_a} and {label_b} never overlap in time')
                continue
            enu_b_at_a = np.array([np.interp(t_a[overlap], t_b, c) for c in enu_b]) # Resample b onto a's timestamps
            ax_dist.plot(t_a[overlap], np.linalg.norm(enu_a[:, overlap] - enu_b_at_a, axis=0), alpha=0.8, label=f'{label_a} - {label_b}')
        ax_dist.set_ylabel('Distance [m]')
        ax_dist.set_xlabel('Time [s]')
        ax_dist.legend(fontsize=8)
    plot_file = os.path.join(log_dir, 'flight_summary.png')
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f'Saved: {plot_file}')
    warnings.filterwarnings('ignore', message='constrained_layout not applied') # Only fires if the interactive window opens too small, the saved PNG is unaffected
    plt.show()
