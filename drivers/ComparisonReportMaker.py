# ===================== PROGRAM_INFO ==================================================================================
"""
Author: Renzo Eisma
Date: 05/2026
Description: This program is for calculating and visualizing the error between UWB measurements and ground truth
measurements.

This rewritten version focuses on the interactive HTML report. The old PDF/static Matplotlib report code was removed
because the HTML report is the main report used for testing.
"""
# =====================================================================================================================


# =====================================================================================================================
# IMPORTS
# =====================================================================================================================
import os
import json
import tkinter as tk
from tkinter import filedialog
from datetime import datetime

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.interpolate import interp1d
from scipy.optimize import minimize


# =====================================================================================================================
# CONFIGURATION
# =====================================================================================================================

# 1. Report Info
measurement_name = '-'
measurement_notes = '-'

REPORT_INFO = {
    'name': measurement_name,
    'notes': measurement_notes
}

# 2. Define your datasets here.
# If one is found in the selected file it will be graphed
# If a signal is selected as ground truth the other signals will be compared to it
# Offsets can be defined
DATASETS = [
    # Ground Truths
    # -----------------------------------------------------------------------------------------------------------------
    {
        'prefix': '[Log]_optitrack',
        'label': 'OptiTrack (GT)',
        'color': 'red',
        'style': '--',
        'is_ground_truth': True,
        'offset': [0, 0, 0],  # Tripods (UWB 000 == Opti 000)
        'multiplier': [1.0, 1.0, 1.0],  # Tripods (UWB 000 == Opti 000)
        'time_offset': 0,
        # 'offset': [4.2604, 3.5112, -0.1],  # Wall Anchors (UWB 000 != Opti 000)
        # 'multiplier': [-1.0, -1.0, 1.0],  # Wall Anchors (UWB 000 != Opti 000)
    },

    # UWB network 1
    # -----------------------------------------------------------------------------------------------------------------
    {
        'prefix': '[Log]_uwb_listener1',
        'label': 'UWB Raw 1',
        'color': '#1f77b4',
        'style': '-',
        'is_ground_truth': False,
        'offset': [0, 0, 0],
        'multiplier': [1.0, 1.0, 1.0],
        'time_offset': 0,
    },
    {
        'prefix': '[Log]_uwbFiltered_listener1',
        'label': 'UWB Filt 1',
        'color': '#2ca02c',
        'style': '--',
        'is_ground_truth': False,
        'offset': [0, 0, 0],
        'multiplier': [1.0, 1.0, 1.0],
        'time_offset': 0,
    },
    {
        'prefix': '[Log]_uwbFilteredMatlab_listener1',
        'label': 'UWB FilteredM 1',
        'color': '#2ca02c',
        'style': '-',
        'is_ground_truth': False,
        'offset': [0, 0, 0],
        'multiplier': [1.0, 1.0, 1.0],
        'time_offset': 0,
    },

    # UWB network 2
    # -----------------------------------------------------------------------------------------------------------------
    {
        'prefix': '[Log]_uwb_listener2',
        'label': 'UWB Raw 2',
        'color': 'yellow',
        'style': '-',
        'is_ground_truth': False,
        'offset': [4.2604, 3.5112, -1.24],
        'multiplier': [-1.0, -1.0, 1.0],
        'time_offset': -0.5,
    },
]

# 3. Features Configuration
DT = 0.1  # Measurement interval (10Hz = 0.1 seconds)
SHOW_ANCHORS = True  # Toggle to show/hide UWB anchors on the map
USE_MEASUREMENT_WINDOW = True
MEASUREMENT_WINDOW_PREFIX = "[Log]_measurement_window"


# =====================================================================================================================
# FUNCTIONS
# =====================================================================================================================

# Generates a graphical popup windows using Tkinter to ask the user for report configurations
# This is so that this script can be used without the MasterControlStation GUI
# ---------------------------------------------------------------------------------------------------------------------
class SettingsDialog(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Report Configuration")
        self.result = None

        tk.Label(self, text="Measurement Name:").grid(row=0, column=0, padx=10, pady=5, sticky="w")
        self.name_entry = tk.Entry(self, width=40)
        self.name_entry.insert(0, REPORT_INFO['name'])
        self.name_entry.grid(row=0, column=1, padx=10, pady=5)

        tk.Label(self, text="Notes:").grid(row=1, column=0, padx=10, pady=5, sticky="w")
        self.notes_entry = tk.Entry(self, width=40)
        self.notes_entry.insert(0, REPORT_INFO['notes'])
        self.notes_entry.grid(row=1, column=1, padx=10, pady=5)

        self.show_anchors_var = tk.BooleanVar(value=SHOW_ANCHORS)
        tk.Checkbutton(self, text="Show Anchors", variable=self.show_anchors_var).grid(
            row=2, column=0, sticky="w", padx=10
        )

        self.use_measurement_window_var = tk.BooleanVar(value=USE_MEASUREMENT_WINDOW)
        tk.Checkbutton(
            self,
            text="Use Measurement Window CSV",
            variable=self.use_measurement_window_var
        ).grid(
            row=3, column=0, sticky="w", padx=10
        )

        tk.Button(self, text="Start Analysis", command=self.on_submit, width=20, bg="lime").grid(
            row=4, column=0, columnspan=2, pady=15
        )

    def on_submit(self):
        self.result = {
            'name': self.name_entry.get(),
            'notes': self.notes_entry.get(),
            'show_anchors': self.show_anchors_var.get(),
            'use_measurement_window': self.use_measurement_window_var.get(),
        }
        self.destroy()


# Finds the most recent log file by searching the directory manually
# This is not used currently
# ---------------------------------------------------------------------------------------------------------------------
def get_latest_file(prefix, folder_path):
    if not os.path.exists(folder_path):
        return None

    # Get all files in the directory
    all_files = os.listdir(folder_path)

    # Filter for files that start with our prefix (e.g., "[Log]_optitrack") and end with .csv
    matching_files = [
        os.path.join(folder_path, f)
        for f in all_files
        if f.startswith(prefix) and f.endswith(".csv")
    ]

    if not matching_files:
        return None

    # Return the most recent one
    return max(matching_files)


# Loads the measurement start/stop timestamps written by MasterControlStation.
# The timestamps are absolute PC timestamps, just like the sensor CSV files.
# ---------------------------------------------------------------------------------------------------------------------
def load_measurement_window(folder_path):
    window_file = get_latest_file(MEASUREMENT_WINDOW_PREFIX, folder_path)

    if window_file is None:
        print("[REPORT] No measurement window file found. Full dataset will be used.")
        return None, None

    try:
        df = pd.read_csv(window_file)

        if "PC_Timestamp" in df.columns:
            time_col = "PC_Timestamp"
        elif "Time" in df.columns:
            time_col = "Time"
        else:
            print("[REPORT] Measurement window file has no PC_Timestamp or Time column. Full dataset will be used.")
            return None, None

        df["Event"] = df["Event"].astype(str).str.lower()
        df[time_col] = pd.to_numeric(df[time_col], errors="coerce")
        df = df.dropna(subset=[time_col])

        start_rows = df[df["Event"] == "start"]
        stop_rows = df[df["Event"] == "stop"]

        if start_rows.empty or stop_rows.empty:
            print("[REPORT] Measurement window file is incomplete. Full dataset will be used.")
            return None, None

        window_start = float(start_rows.iloc[0][time_col])
        window_stop = float(stop_rows.iloc[-1][time_col])

        if window_stop <= window_start:
            print("[REPORT] Measurement window stop time is before start time. Full dataset will be used.")
            return None, None

        print(f"[REPORT] Measurement window loaded:")
        print(f"         Start: {window_start:.5f}")
        print(f"         Stop : {window_stop:.5f}")

        return window_start, window_stop

    except Exception as e:
        print(f"[REPORT] Could not read measurement window file. Error: {e}")
        return None, None


# Crops one dataset to the measurement window.
# This should be called after coordinate offsets and time_offset are applied.
# ---------------------------------------------------------------------------------------------------------------------
def apply_measurement_window(df, window_start, window_stop, label):
    if df is None or df.empty:
        return df

    if window_start is None or window_stop is None:
        return df

    before_count = len(df)

    df = df[
        (df["pc_timestamp"] >= window_start) &
        (df["pc_timestamp"] <= window_stop)
    ].copy()

    after_count = len(df)

    print(f"[REPORT] {label}: kept {after_count}/{before_count} samples inside measurement window.")

    if df.empty:
        print(f"[REPORT] WARNING: {label} has no samples inside the measurement window.")

    return df.reset_index(drop=True)


# This function takes a file path and reads the CSV data using the pandas library.
# ---------------------------------------------------------------------------------------------------------------------
def load_position_data(filepath):
    try:
        df = pd.read_csv(filepath)

        # Standardize column names for internal use
        df.columns = ['pc_timestamp', 'x', 'y', 'z']
        return df.apply(pd.to_numeric, errors='coerce').dropna().reset_index(drop=True)

    except Exception as e:
        print(f"WARNING: Could not load {filepath}. Error: {e}")
        return None


# This function computes how fast the tracked object is moving. It does this by calculating the physical distance
# between consecutive 3D points and dividing that distance by the time elapsed between those specific measurements.
# ---------------------------------------------------------------------------------------------------------------------
def calculate_velocity(df):
    if df is None or len(df) < 2:
        return np.zeros(0)

    coords = df[['x', 'y', 'z']].values
    times = df['pc_timestamp'].values

    diffs = np.diff(coords, axis=0)
    dt = np.diff(times)

    # Avoid division by zero if two points have the same timestamp
    dt[dt == 0] = 0.001

    dists = np.linalg.norm(diffs, axis=1)
    return np.insert(dists / dt, 0, 0.0)


# This function stands for Absolute Trajectory Error. It compares the estimated UWB positions against the actual ground
# truth positions. It calculates the overall 3D distances between the points to find the mean error, maximum error, and
# root-mean-square error. It also calculates the error for the individual X, Y, and Z axes and returns a formatted list
# of strings to be displayed in the final report table.
# ---------------------------------------------------------------------------------------------------------------------
def calculate_ate(df_est, df_gt, label):
    """Calculates the Error and returns formatting for the table."""
    min_len = min(len(df_est), len(df_gt))
    if min_len == 0:
        return None

    est_pts = df_est[['x', 'y', 'z']].values[:min_len]
    gt_pts = df_gt[['x', 'y', 'z']].values[:min_len]

    # Calculate 3D Euclidean error
    errors = np.linalg.norm(est_pts - gt_pts, axis=1)

    mae_3d = np.mean(errors) * 100
    rmse_3d = np.sqrt(np.mean(errors ** 2)) * 100
    p95_3d = np.percentile(errors, 95) * 100

    return [label, f"{mae_3d:.2f} cm", f"{rmse_3d:.2f} cm", f"{p95_3d:.2f} cm"]


# This function acts as an auto calibration tool. It takes the UWB data and the ground truth data and uses a
# mathematical optimization algorithm called Nelder Mead. The algorithm tests different spatial shifts for X, Y, and Z,
# as well as time shifts, to find the exact offset values that make the two datasets align as good as possible. It then
# prints these suggested offset values to the console.
# ---------------------------------------------------------------------------------------------------------------------
def find_best_alignment(df_uwb, df_gt, label="UWB"):
    # 1. Normalize time to start at 0
    uwb_t = (df_uwb['pc_timestamp'] - df_uwb['pc_timestamp'].iloc[0]).values
    gt_t = (df_gt['pc_timestamp'] - df_gt['pc_timestamp'].iloc[0]).values

    uwb_pts = df_uwb[['x', 'y', 'z']].values
    gt_pts = df_gt[['x', 'y', 'z']].values

    # Interpolator for Ground Truth (so we can sample it at any time)
    interp_gt = interp1d(gt_t, gt_pts, axis=0, bounds_error=False, fill_value="extrapolate")

    def objective_function(params):
        dx, dy, dz, dt = params

        # Apply time shift and spatial offset
        shifted_t = uwb_t + dt
        shifted_uwb = uwb_pts + [dx, dy, dz]

        # Sample Ground Truth at the shifted UWB times
        sampled_gt = interp_gt(shifted_t)

        # Calculate Euclidean Error
        error = np.linalg.norm(shifted_uwb - sampled_gt, axis=1)
        return np.mean(error ** 2)

    # Initial guess: [0 offset x, 0 offset y, 0 offset z, 0 time shift]
    initial_guess = [0, 0, 0, 0]
    res = minimize(objective_function, initial_guess, method='Nelder-Mead')
    dx, dy, dz, dt = res.x

    print(f"\nOptimization results for: {label}")
    print("-" * 40)
    print(f"Suggested 'offset': [{dx:.4f}, {dy:.4f}, {dz:.4f}]")
    print(f"Suggested 'time_offset': {dt:.4f}")
    print("-" * 40)

    # Return the values but won't force them into the data automatically
    return dx, dy, dz, dt


# This function synchronizes one measured dataset to the ground truth data. Ground truth is interpolated at the
# measurement timestamps. The signed axis errors are calculated in centimeters and the absolute magnitude errors are
# calculated from those signed errors.
# ---------------------------------------------------------------------------------------------------------------------
def calculate_error_dataframe(df_est, df_gt):
    if df_est is None or df_gt is None or df_est.empty or df_gt.empty:
        return pd.DataFrame()

    df_est = df_est.sort_values('pc_timestamp').reset_index(drop=True)
    df_gt = df_gt.sort_values('pc_timestamp').reset_index(drop=True)

    # Interpolation instead of merge_asof for better accuracy
    f_x = interp1d(df_gt['pc_timestamp'], df_gt['x'], bounds_error=False, fill_value="extrapolate")
    f_y = interp1d(df_gt['pc_timestamp'], df_gt['y'], bounds_error=False, fill_value="extrapolate")
    f_z = interp1d(df_gt['pc_timestamp'], df_gt['z'], bounds_error=False, fill_value="extrapolate")

    gt_interp_x = f_x(df_est['pc_timestamp'])
    gt_interp_y = f_y(df_est['pc_timestamp'])
    gt_interp_z = f_z(df_est['pc_timestamp'])

    err_x_signed = (df_est['x'].values - gt_interp_x) * 100
    err_y_signed = (df_est['y'].values - gt_interp_y) * 100
    err_z_signed = (df_est['z'].values - gt_interp_z) * 100

    err_3d = np.sqrt(err_x_signed ** 2 + err_y_signed ** 2 + err_z_signed ** 2)
    err_xy = np.sqrt(err_x_signed ** 2 + err_y_signed ** 2)
    err_z_abs = np.abs(err_z_signed)

    return pd.DataFrame({
        'pc_timestamp': df_est['pc_timestamp'].values,
        'x_est': df_est['x'].values,
        'y_est': df_est['y'].values,
        'z_est': df_est['z'].values,
        'x_gt': gt_interp_x,
        'y_gt': gt_interp_y,
        'z_gt': gt_interp_z,
        'x_error_signed_cm': err_x_signed,
        'y_error_signed_cm': err_y_signed,
        'z_error_signed_cm': err_z_signed,
        'error_3d_cm': err_3d,
        'error_xy_cm': err_xy,
        'error_z_abs_cm': err_z_abs,
    })


# This function calculates the statistical values shown in the summary table.
# ---------------------------------------------------------------------------------------------------------------------
def calculate_summary_metrics(error_df):
    if error_df is None or error_df.empty:
        return None

    x_err = error_df['x_error_signed_cm'].values
    y_err = error_df['y_error_signed_cm'].values
    z_err = error_df['z_error_signed_cm'].values
    err_3d = error_df['error_3d_cm'].values
    err_xy = error_df['error_xy_cm'].values
    err_z_abs = error_df['error_z_abs_cm'].values

    return {
        '3D MAE': np.mean(err_3d),
        '3D RMSE': np.sqrt(np.mean(err_3d ** 2)),
        '3D P95': np.percentile(err_3d, 95),
        'XY RMSE': np.sqrt(np.mean(err_xy ** 2)),
        'Z RMSE': np.sqrt(np.mean(z_err ** 2)),
        'XY MAE': np.mean(err_xy),
        'Z MAE': np.mean(err_z_abs),
        'X bias': np.mean(x_err),
        'Y bias': np.mean(y_err),
        'Z bias': np.mean(z_err),
        'X std': np.std(x_err),
        'Y std': np.std(y_err),
        'Z std': np.std(z_err),
    }


# This function interpolates the ground truth speed at the timestamps of the dataset being compared. This makes the
# velocity-vs-error plot use the ground truth speed instead of speed calculated from noisy UWB position data.
# ---------------------------------------------------------------------------------------------------------------------
def calculate_ground_truth_speed_at_measurement_times(df_gt, measurement_times):
    if df_gt is None or df_gt.empty or len(df_gt) < 2:
        return np.zeros(len(measurement_times))

    gt_speed = calculate_velocity(df_gt)
    f_speed = interp1d(df_gt['pc_timestamp'], gt_speed, bounds_error=False, fill_value="extrapolate")
    return f_speed(measurement_times)


# This function adds vertical percentile lines to a CDF plot. The lines are drawn for P50, P90 and P95.
# ---------------------------------------------------------------------------------------------------------------------
def add_percentile_lines(fig, errors, row, col, color, label):
    if errors is None or len(errors) == 0:
        return

    percentile_settings = [
        (50, 'P50', 'dot'),
        (90, 'P90', 'dash'),
        (95, 'P95', 'solid'),
    ]

    for percentile, percentile_name, dash_style in percentile_settings:
        value = np.percentile(errors, percentile)
        fig.add_trace(
            go.Scatter(
                x=[value, value],
                y=[0, 1],
                mode='lines',
                name=f"{label} {percentile_name}",
                line=dict(color=color, dash=dash_style, width=1),
                showlegend=False,
                hovertemplate=f"{label}<br>{percentile_name}: {value:.2f} cm<extra></extra>",
            ),
            row=row,
            col=col,
        )


# =====================================================================================================================
# This function builds a fully interactive web based dashboard. It uses the Plotly library to create a grid containing
# a 3D trajectory map, a 2D top down view, individual axis position graphs, individual signed error graphs, total 3D
# and XY error graphs, CDF plots, velocity-vs-error diagnostics, sample timing, and a statistical error table. It
# synchronizes the time across all datasets, calculates live error metrics, plots the UWB anchor locations, and exports
# the entire interactive figure as an HTML file.
# =====================================================================================================================
def generate_plotly_dashboard(loaded_data, session_folder, report_name):
    """
    Creates an interactive HTML dashboard with 3D, 2D, time-series plots, advanced diagnostics, and an error table.
    """

    # Create a 9x2 grid of subplots
    fig = make_subplots(
        rows=9,
        cols=2,
        specs=[
            [{'type': 'scene'}, {'type': 'xy'}],          # Row 1: 3D Path | 2D Top Down
            [{'type': 'xy'}, {'type': 'xy'}],             # Row 2: X Position | X Error
            [{'type': 'xy'}, {'type': 'xy'}],             # Row 3: Y Position | Y Error
            [{'type': 'xy'}, {'type': 'xy'}],             # Row 4: Z Position | Z Error
            [{'type': 'xy'}, {'type': 'xy'}],             # Row 5: 3D Error | XY Error
            [{'type': 'xy'}, {'type': 'xy'}],             # Row 6: 3D CDF | XY CDF
            [{'type': 'xy'}, {'type': 'xy'}],             # Row 7: Z CDF | Velocity vs Error
            [{'type': 'xy', 'colspan': 2}, None],         # Row 8: Sample Timing
            [{'type': 'table', 'colspan': 2}, None],      # Row 9: Error Metrics Table
        ],
        subplot_titles=(
            "3D Trajectory Map",
            "2D Top-Down View",
            "X-Axis Position",
            "X-Axis Signed Error",
            "Y-Axis Position",
            "Y-Axis Signed Error",
            "Z-Axis (Altitude) Position",
            "Z-Axis Signed Error",
            "3D Error Compared to Ground Truth",
            "XY Error Compared to Ground Truth",
            "3D Error Distribution (CDF)",
            "XY Error Distribution (CDF)",
            "Z Error Distribution (CDF)",
            "Ground Truth Speed vs 3D Error",
            "Sample Rate Consistency (Time Between Points)",
            "Statistical Error Summary",
        ),
        vertical_spacing=0.035,
    )

    # Find Ground Truth for Error calculation
    gt_key = next(
        (k for k, v in loaded_data.items() if any(d['label'] == k and d.get('is_ground_truth') for d in DATASETS)),
        None,
    )

    global_start_time = min(df['pc_timestamp'].min() for df in loaded_data.values())

    # Data structures for the Table
    table_headers = [
        'Sensor',
        '3D MAE',
        '3D RMSE',
        '3D P95',
        'XY RMSE',
        'Z RMSE',
        'XY MAE',
        'Z MAE',
        'X bias',
        'Y bias',
        'Z bias',
        'X std',
        'Y std',
        'Z std',
    ]
    table_cells = [[] for _ in table_headers]

    for lbl, df in loaded_data.items():
        time_index = df['pc_timestamp'] - global_start_time
        color = next((d['color'] for d in DATASETS if d['label'] == lbl), 'gray')

        # 1. 3D Trajectory
        fig.add_trace(
            go.Scatter3d(
                x=df['x'],
                y=df['y'],
                z=df['z'],
                mode='lines',
                name=lbl,
                line=dict(color=color, width=4),
                hovertemplate="Time: %{text}s<br>X: %{x:.2f} m<br>Y: %{y:.2f} m<br>Z: %{z:.2f} m<extra></extra>",
                text=[f"{t:.2f}" for t in time_index],
            ),
            row=1,
            col=1,
        )

        # 2. 2D Top Down
        fig.add_trace(
            go.Scatter(
                x=df['x'],
                y=df['y'],
                mode='lines',
                name=lbl,
                line=dict(color=color),
                showlegend=False,
                hovertemplate="X: %{x:.2f} m<br>Y: %{y:.2f} m<extra></extra>",
            ),
            row=1,
            col=2,
        )

        # 3. X Position
        fig.add_trace(
            go.Scatter(
                x=time_index,
                y=df['x'],
                mode='lines',
                name=lbl,
                line=dict(color=color),
                showlegend=False,
                hovertemplate="Time: %{x:.2f}s<br>X: %{y:.2f} m<extra></extra>",
            ),
            row=2,
            col=1,
        )

        # 4. Y Position
        fig.add_trace(
            go.Scatter(
                x=time_index,
                y=df['y'],
                mode='lines',
                name=lbl,
                line=dict(color=color),
                showlegend=False,
                hovertemplate="Time: %{x:.2f}s<br>Y: %{y:.2f} m<extra></extra>",
            ),
            row=3,
            col=1,
        )

        # 5. Z Position
        fig.add_trace(
            go.Scatter(
                x=time_index,
                y=df['z'],
                mode='lines',
                name=lbl,
                line=dict(color=color),
                showlegend=False,
                hovertemplate="Time: %{x:.2f}s<br>Z: %{y:.2f} m<extra></extra>",
            ),
            row=4,
            col=1,
        )

        # 6. Sample Rate Consistency (Delta T)
        deltas = df['pc_timestamp'].diff().dropna()
        fig.add_trace(
            go.Scatter(
                x=time_index[1:],
                y=deltas,
                mode='lines',
                name=f"{lbl} Timing",
                line=dict(color=color, width=1),
                showlegend=False,
                hovertemplate="Time: %{x:.2f}s<br>Delta: %{y:.4f} s<extra></extra>",
            ),
            row=8,
            col=1,
        )

        # 7. Error Calculation (relative to GT)
        if gt_key and lbl != gt_key:
            df_gt = loaded_data[gt_key]
            error_df = calculate_error_dataframe(df, df_gt)
            if error_df.empty:
                continue

            error_time_index = error_df['pc_timestamp'] - global_start_time

            # --- Populate Table Data ---
            metrics = calculate_summary_metrics(error_df)
            if metrics:
                table_cells[0].append(lbl)
                for metric_name in table_headers[1:]:
                    table_cells[table_headers.index(metric_name)].append(f"{metrics[metric_name]:.1f} cm")

            # X Signed Error
            fig.add_trace(
                go.Scatter(
                    x=error_time_index,
                    y=error_df['x_error_signed_cm'],
                    mode='lines',
                    name=f"{lbl} (X Err)",
                    line=dict(color=color, width=2),
                    showlegend=False,
                    hovertemplate="Time: %{x:.2f}s<br>X Signed Error: %{y:.2f} cm<extra></extra>",
                ),
                row=2,
                col=2,
            )

            # Y Signed Error
            fig.add_trace(
                go.Scatter(
                    x=error_time_index,
                    y=error_df['y_error_signed_cm'],
                    mode='lines',
                    name=f"{lbl} (Y Err)",
                    line=dict(color=color, width=2),
                    showlegend=False,
                    hovertemplate="Time: %{x:.2f}s<br>Y Signed Error: %{y:.2f} cm<extra></extra>",
                ),
                row=3,
                col=2,
            )

            # Z Signed Error
            fig.add_trace(
                go.Scatter(
                    x=error_time_index,
                    y=error_df['z_error_signed_cm'],
                    mode='lines',
                    name=f"{lbl} (Z Err)",
                    line=dict(color=color, width=2),
                    showlegend=False,
                    hovertemplate="Time: %{x:.2f}s<br>Z Signed Error: %{y:.2f} cm<extra></extra>",
                ),
                row=4,
                col=2,
            )

            # Total 3D Error
            fig.add_trace(
                go.Scatter(
                    x=error_time_index,
                    y=error_df['error_3d_cm'],
                    mode='lines',
                    name=f"{lbl} (3D Err)",
                    line=dict(color=color, width=2),
                    hovertemplate="Time: %{x:.2f}s<br>3D Error: %{y:.2f} cm<extra></extra>",
                ),
                row=5,
                col=1,
            )

            # XY Error
            fig.add_trace(
                go.Scatter(
                    x=error_time_index,
                    y=error_df['error_xy_cm'],
                    mode='lines',
                    name=f"{lbl} (XY Err)",
                    line=dict(color=color, width=2),
                    showlegend=False,
                    hovertemplate="Time: %{x:.2f}s<br>XY Error: %{y:.2f} cm<extra></extra>",
                ),
                row=5,
                col=2,
            )

            # 8. 3D CDF Plot
            sorted_err_3d = np.sort(error_df['error_3d_cm'])
            y_vals_3d = np.arange(len(sorted_err_3d)) / float(len(sorted_err_3d))
            fig.add_trace(
                go.Scatter(
                    x=sorted_err_3d,
                    y=y_vals_3d,
                    mode='lines',
                    name=f"{lbl} 3D CDF",
                    line=dict(color=color),
                    hovertemplate="3D Error: %{x:.2f} cm<br>Probability: %{y:.2f}<extra></extra>",
                ),
                row=6,
                col=1,
            )
            add_percentile_lines(fig, sorted_err_3d, row=6, col=1, color=color, label=lbl)

            # 9. XY CDF Plot
            sorted_err_xy = np.sort(error_df['error_xy_cm'])
            y_vals_xy = np.arange(len(sorted_err_xy)) / float(len(sorted_err_xy))
            fig.add_trace(
                go.Scatter(
                    x=sorted_err_xy,
                    y=y_vals_xy,
                    mode='lines',
                    name=f"{lbl} XY CDF",
                    line=dict(color=color),
                    showlegend=False,
                    hovertemplate="XY Error: %{x:.2f} cm<br>Probability: %{y:.2f}<extra></extra>",
                ),
                row=6,
                col=2,
            )
            add_percentile_lines(fig, sorted_err_xy, row=6, col=2, color=color, label=lbl)

            # 10. Z CDF Plot
            sorted_err_z = np.sort(error_df['error_z_abs_cm'])
            y_vals_z = np.arange(len(sorted_err_z)) / float(len(sorted_err_z))
            fig.add_trace(
                go.Scatter(
                    x=sorted_err_z,
                    y=y_vals_z,
                    mode='lines',
                    name=f"{lbl} Z CDF",
                    line=dict(color=color),
                    showlegend=False,
                    hovertemplate="Z Error: %{x:.2f} cm<br>Probability: %{y:.2f}<extra></extra>",
                ),
                row=7,
                col=1,
            )
            add_percentile_lines(fig, sorted_err_z, row=7, col=1, color=color, label=lbl)

            # 11. Velocity vs Error
            gt_speed = calculate_ground_truth_speed_at_measurement_times(df_gt, error_df['pc_timestamp'].values)
            fig.add_trace(
                go.Scatter(
                    x=gt_speed,
                    y=error_df['error_3d_cm'],
                    mode='markers',
                    marker=dict(size=4, opacity=0.5, color=color),
                    name=f"{lbl} GT Speed vs Error",
                    showlegend=False,
                    hovertemplate="Ground Truth Speed: %{x:.2f} m/s<br>3D Error: %{y:.2f} cm<extra></extra>",
                ),
                row=7,
                col=2,
            )

    # 12. Draw the Table
    if table_cells[0]:
        fig.add_trace(
            go.Table(
                header=dict(
                    values=table_headers,
                    fill_color='paleturquoise',
                    align='center',
                    font=dict(size=11, color='black'),
                ),
                cells=dict(
                    values=table_cells,
                    fill_color='lavender',
                    align='center',
                    font=dict(size=10, color='black'),
                ),
            ),
            row=9,
            col=1,
        )

    # Plot Anchors in 3D and 2D
    if SHOW_ANCHORS:
        anchor_csv = os.path.join(session_folder, "[Log]_anchor_positions.csv")
        if os.path.exists(anchor_csv):
            try:
                anchor_df = pd.read_csv(anchor_csv)
                current_anchors = anchor_df[['X', 'Y', 'Z']].values.tolist()
                print(f"Successfully loaded {len(current_anchors)} anchors from CSV.")

                an = np.array(current_anchors)
                fig.add_trace(
                    go.Scatter3d(
                        x=an[:, 0],
                        y=an[:, 1],
                        z=an[:, 2],
                        mode='markers',
                        marker=dict(symbol='diamond', size=5, color='lime'),
                        name='Anchors',
                        hovertemplate="Anchor<br>X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>",
                    ),
                    row=1,
                    col=1,
                )

                fig.add_trace(
                    go.Scatter(
                        x=an[:, 0],
                        y=an[:, 1],
                        mode='markers',
                        marker=dict(symbol='diamond', size=10, color='lime'),
                        name='Anchors',
                        showlegend=False,
                        hovertemplate="Anchor<br>X: %{x:.2f}<br>Y: %{y:.2f}<extra></extra>",
                    ),
                    row=1,
                    col=2,
                )

            except Exception as e:
                print(f"Error reading anchor CSV: {e}")

    # =================================================================================================================
    # LABEL ALL AXES
    # =================================================================================================================

    # Row 1: 3D Map
    fig.update_layout(
        scene=dict(
            xaxis_title='X Position (m)',
            yaxis_title='Y Position (m)',
            zaxis_title='Z Altitude (m)',
        )
    )

    # Row 1: 2D Map (Scale anchored to keep aspect ratio 1:1)
    fig.update_xaxes(title_text="X Position (m)", row=1, col=2)
    fig.update_yaxes(title_text="Y Position (m)", scaleanchor="x", scaleratio=1, row=1, col=2)

    # Row 2: X Position and X Signed Error
    fig.update_xaxes(title_text="Elapsed Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="X Position (m)", row=2, col=1)
    fig.update_xaxes(title_text="Elapsed Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="X Signed Error (cm)", row=2, col=2)

    # Row 3: Y Position and Y Signed Error
    fig.update_xaxes(title_text="Elapsed Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Y Position (m)", row=3, col=1)
    fig.update_xaxes(title_text="Elapsed Time (s)", row=3, col=2)
    fig.update_yaxes(title_text="Y Signed Error (cm)", row=3, col=2)

    # Row 4: Z Position and Z Signed Error
    fig.update_xaxes(title_text="Elapsed Time (s)", row=4, col=1)
    fig.update_yaxes(title_text="Z Altitude (m)", row=4, col=1)
    fig.update_xaxes(title_text="Elapsed Time (s)", row=4, col=2)
    fig.update_yaxes(title_text="Z Signed Error (cm)", row=4, col=2)

    # Row 5: 3D Error and XY Error over time
    fig.update_xaxes(title_text="Elapsed Time (s)", row=5, col=1)
    fig.update_yaxes(title_text="3D Error (cm)", row=5, col=1)
    fig.update_xaxes(title_text="Elapsed Time (s)", row=5, col=2)
    fig.update_yaxes(title_text="XY Error (cm)", row=5, col=2)

    # Row 6: CDF 3D and CDF XY
    fig.update_xaxes(title_text="3D Error magnitude (cm)", row=6, col=1)
    fig.update_yaxes(title_text="Cumulative Probability", row=6, col=1)
    fig.update_xaxes(title_text="XY Error magnitude (cm)", row=6, col=2)
    fig.update_yaxes(title_text="Cumulative Probability", row=6, col=2)

    # Row 7: CDF Z and Velocity vs Error
    fig.update_xaxes(title_text="Z Error magnitude (cm)", row=7, col=1)
    fig.update_yaxes(title_text="Cumulative Probability", row=7, col=1)
    fig.update_xaxes(title_text="Ground Truth Speed / Velocity (m/s)", row=7, col=2)
    fig.update_yaxes(title_text="3D Error (cm)", row=7, col=2)

    # Row 8: Sample Rate
    fig.update_xaxes(title_text="Elapsed Time (s)", row=8, col=1)
    fig.update_yaxes(title_text="Delta T (Seconds between points)", row=8, col=1)

    # Generate the Title with Export Date and Notes
    current_date = datetime.now().strftime('%Y-%m-%d %H:%M')
    notes = REPORT_INFO['notes']
    title_html = f"Interactive Report: {report_name}<br>Export Date: {current_date} | Notes: {notes}"

    # Increase total height to give the table and extra plots enough breathing room
    fig.update_layout(
        height=3600,
        title_text=title_html,
        template="plotly_white",
        margin=dict(t=120),
    )

    safe_name = f"[Report]_{REPORT_INFO['name'].replace(' ', '_')}.html"
    html_path = os.path.join(session_folder, safe_name)
    fig.write_html(html_path)
    print(f" -> Interactive Plotly report saved: {html_path}")


# =====================================================================================================================
# This is the master function that orchestrates the entire plotting process. It handles fetching the target folder,
# either through a popup dialog or a saved configuration file. It then loops through the configured datasets, loads the
# latest CSV files, applies any manual coordinate or time offsets, and normalizes the timestamps. It calls the
# interactive dashboard generator and attempts to rename the original folder to match the finalized report name.
# =====================================================================================================================
def run_dashboard(session_folder=None, skip_popup=False):
    global REPORT_INFO, SHOW_ANCHORS, USE_MEASUREMENT_WINDOW

    # ONLY create a new Tkinter root if we are running completely standalone
    root = None
    if not skip_popup:
        root = tk.Tk()
        root.withdraw()

    # If no folder was provided, ask the user
    if session_folder is None:
        if not skip_popup:
            session_folder = filedialog.askdirectory(title="Select Measurement Session Folder")
        if not session_folder:
            print("No folder selected. Exiting.")
            return

    settings_path = os.path.join(session_folder, "report_settings.json")

    # If triggered by Master Control Station, read JSON and skip popup
    if skip_popup and os.path.exists(settings_path):
        try:
            with open(settings_path, 'r') as f:
                config = json.load(f)

            REPORT_INFO['name'] = config.get('name', REPORT_INFO['name'])
            REPORT_INFO['notes'] = config.get('notes', REPORT_INFO['notes'])
            SHOW_ANCHORS = config.get('show_anchors', SHOW_ANCHORS)
            USE_MEASUREMENT_WINDOW = config.get('use_measurement_window', USE_MEASUREMENT_WINDOW)
            print("Successfully loaded report settings from JSON.")

        except Exception as e:
            print(f"Error reading report_settings.json: {e}")

    # Otherwise, show the traditional popup (for standalone use)
    else:
        if root is not None:
            dialog = SettingsDialog(root)
            root.wait_window(dialog)

            if not dialog.result:
                print("Configuration cancelled.")
                return

            REPORT_INFO['name'] = dialog.result['name']
            REPORT_INFO['notes'] = dialog.result['notes']
            SHOW_ANCHORS = dialog.result['show_anchors']
            USE_MEASUREMENT_WINDOW = dialog.result['use_measurement_window']

    window_start = None
    window_stop = None

    if USE_MEASUREMENT_WINDOW:
        window_start, window_stop = load_measurement_window(session_folder)
    else:
        print("[REPORT] Measurement window cropping disabled.")

    loaded_data = {}
    gt_key = None

    # 1. Load Data First
    # -----------------------------------------------------------------------------------------------------------------
    for ds in DATASETS:
        # AUTOMATICALLY find the latest file based on the prefix
        filepath = get_latest_file(ds['prefix'], session_folder)
        if not filepath:
            print(f"WARNING: No file found for {ds['prefix']}")
            continue

        df = load_position_data(filepath)
        if df is None or df.empty:
            continue

        print(f"Processing latest file: {filepath} for {ds['label']}")

        # --- APPLY COORDINATE OFFSET ---
        if 'multiplier' in ds:
            mult = ds['multiplier']
            df['x'] *= mult[0]
            df['y'] *= mult[1]
            df['z'] *= mult[2]

        if 'offset' in ds:
            off = ds['offset']
            df['x'] += off[0]
            df['y'] += off[1]
            df['z'] += off[2]

        t_off = ds.get('time_offset', 0.0)
        df['pc_timestamp'] += t_off

        lbl = ds['label']

        df = apply_measurement_window(df, window_start, window_stop, lbl)

        if df is None or df.empty:
            continue

        loaded_data[lbl] = df

        if ds.get('is_ground_truth'):
            gt_key = lbl

    if not loaded_data:
        print("No data loaded. Exiting.")
        return

    if gt_key and gt_key in loaded_data:
        for lbl, df in loaded_data.items():
            if lbl != gt_key and "UWB Raw" in lbl:
                # This just prints the suggestions to the console
                find_best_alignment(df, loaded_data[gt_key], label=lbl)
                break

    generate_plotly_dashboard(loaded_data, session_folder, REPORT_INFO['name'])

    # 1. Get the parent directory (e.g., 'measurements/')
    parent_dir = os.path.dirname(session_folder)

    # 2. Construct the new path
    report_title = f"[Report]_{REPORT_INFO['name'].replace(' ', '_')}"
    new_folder_path = os.path.join(parent_dir, report_title)

    # 3. Rename the folder safely
    try:
        if os.path.abspath(session_folder) != os.path.abspath(new_folder_path):
            os.rename(session_folder, new_folder_path)
            print(f"Folder successfully renamed to: {report_title}")
    except PermissionError:
        print("Could not rename folder. Make sure no CSV files are open in Excel!")
    except FileExistsError:
        print("Could not rename folder. A folder with this report name already exists!")


if __name__ == "__main__":
    run_dashboard()
