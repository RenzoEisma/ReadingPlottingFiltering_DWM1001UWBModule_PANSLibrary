# ===================== PROGRAM_INFO ==================================================================================
""" Author: Renzo Eisma
    Date: 03/19/2026
    Description: This program is for calculating and visualizing
    the error between UWB measurements and ground truth measurements"""

# =====================================================================================================================
# IMPORTS
# =====================================================================================================================

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from datetime import datetime
import glob
import tkinter as tk
from tkinter import filedialog
import os
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.optimize import minimize
from scipy.interpolate import interp1d
import json # Make sure this is added to your imports at the top of the file

# =====================================================================================================================
# CONFIGURATION
# =====================================================================================================================

measurement_name = 'UWB_TwoAWallTwoACornersMat_Att2'
measurement_notes = '---'

# 1. Report Info
REPORT_INFO = {
    'name': measurement_name,
    'notes': measurement_notes
}

# 2. Define your datasets here.
DATASETS = [
    {
        'prefix': '[Log]_optitrack', 'label': 'OptiTrack (GT)', 'color': 'red', 'style': '--',
        'is_ground_truth': True,
        'offset': [0, 0, 0], # Tripods (UWB 000 == Opti 000)
        'multiplier': [1.0, 1.0, 1.0], # Tripods (UWB 000 == Opti 000)
        'time_offset': 0,
        # 'offset': [4.2604, 3.5112, -0.1], # Wall Anchors (UWB 000 != Opti 000)
        # 'multiplier': [-1.0, -1.0, 1.0], # Wall Anchors (UWB 000 != Opti 000)
    },
    # {
    #     'prefix': '[Log]_uwb','label': 'UWB Raw','color': 'blue','style': '-',
    #     'is_ground_truth': False,
    #     'offset': [0,0,-0.25], #[-4.2604, -3.5112, -0.3094]
    #     'multiplier': [1.0, 1.0, 1.0],
    #     'time_offset': -0.5,
    # },
    # {
    #     'prefix': '[Log]_uwbFiltered','label': 'UWB Filtered','color': 'orange','style': '-',
    #     'is_ground_truth': False,
    #     'offset': [0,0,-0.25],
    #     'multiplier': [1.0, 1.0, 1.0],
    #     'time_offset': -0.5,
    # },
    {
        'prefix': '[Log]_uwb_listener1','label': 'UWB Raw 1','color': 'orange','style': '-',
        'is_ground_truth': False,
        'offset': [0,0,-0.25],
        'multiplier': [1.0, 1.0, 1.0],
        'time_offset': -0.5,
    },
    {
        'prefix': '[Log]_uwb_listener2','label': 'UWB Raw 2','color': 'yellow','style': '-',
        'is_ground_truth': False,
        'offset': [4.2604, 3.5112, -1.24],
        'multiplier': [-1.0, -1.0, 1.0],
        'time_offset': -0.5,
    },
    {
        'prefix': '[Log]_uwbFilteredMatlab_listener1','label': 'UWB FilteredM 1','color': 'yellow','style': '-',
        'is_ground_truth': False,
        'offset': [0,0,-0.25],
        'multiplier': [1.0, 1.0, 1.0],
        'time_offset': -0.5,
    },
]

# 3. Features Configuration
DT = 0.1  # Measurement interval (10Hz = 0.1 seconds)
SHOW_ANCHORS = True  # Toggle to show/hide UWB anchors on the map
SAVE_PDF = True # Save to a PDF

# 4. Individual PNG's
SAVE_INDIVIDUAL_PNGS = True  # Master toggle for PNG exports
EXPORT_LIST = {
    '3d_map': False,
    '2d_top_down': False,
    'z_stability': True,
    'velocity': False,
    'total_error': False,
    'x_axis_comparison': True,
    'y_axis_comparison': True
}



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
        tk.Checkbutton(self, text="Show Anchors", variable=self.show_anchors_var).grid(row=2, column=0, sticky="w", padx=10)

        self.save_pdf_var = tk.BooleanVar(value=SAVE_PDF)
        tk.Checkbutton(self, text="Save PDF Report", variable=self.save_pdf_var).grid(row=2, column=1, sticky="w", padx=10)

        self.save_pngs_var = tk.BooleanVar(value=SAVE_INDIVIDUAL_PNGS)
        tk.Checkbutton(self, text="Save Individual PNGs", variable=self.save_pngs_var).grid(row=3, column=0, sticky="w", padx=10)

        tk.Button(self, text="Start Analysis", command=self.on_submit, width=20, bg="lime").grid(row=4, column=0, columnspan=2, pady=15)

    def on_submit(self):
        self.result = {
            'name': self.name_entry.get(),
            'notes': self.notes_entry.get(),
            'show_anchors': self.show_anchors_var.get(),
            'save_pdf': self.save_pdf_var.get(),
            'save_pngs': self.save_pngs_var.get()
        }
        self.destroy()


def get_latest_file(prefix, folder_path):
    """Finds the most recent log file by searching the directory manually, avoiding glob's bracket issue."""
    if not os.path.exists(folder_path):
        return None

    # Get all files in the directory
    all_files = os.listdir(folder_path)

    # Filter for files that start with our prefix (e.g., "[Log]_optitrack") and end with .csv
    matching_files = [
        os.path.join(folder_path, f) for f in all_files
        if f.startswith(prefix) and f.endswith(".csv")
    ]

    if not matching_files:
        return None

    # Return the most recent one (max works perfectly because your timestamps sort alphabetically)
    return max(matching_files)


def load_position_data(filepath):
    """Loads the new 4-column format: Time, POSX, POSY, POSZ."""
    try:
        df = pd.read_csv(filepath)
        # Standardize column names for internal use
        df.columns = ['pc_timestamp', 'x', 'y', 'z']
        return df.apply(pd.to_numeric, errors='coerce').dropna().reset_index(drop=True)
    except Exception as e:
        print(f"WARNING: Could not load {filepath}. Error: {e}")
        return None


def calculate_velocity(df):
    """Calculates point-to-point speed using the actual time between samples."""
    coords = df[['x', 'y', 'z']].values
    times = df['pc_timestamp'].values

    diffs = np.diff(coords, axis=0)
    dt = np.diff(times)

    # Avoid division by zero if two points have the same timestamp
    dt[dt == 0] = 0.001

    dists = np.linalg.norm(diffs, axis=1)
    return np.insert(dists / dt, 0, 0.0)


def calculate_ate(df_est, df_gt, label):
    """Calculates the Error and returns formatting for the table."""
    min_len = min(len(df_est), len(df_gt))
    if min_len == 0: return None

    est_pts = df_est[['x', 'y', 'z']].values[:min_len]
    gt_pts = df_gt[['x', 'y', 'z']].values[:min_len]

    # Calculate 3D Euclidean error
    errors = np.linalg.norm(est_pts - gt_pts, axis=1)
    mean_err = np.mean(errors) * 100
    max_err = np.max(errors) * 100
    rmse = np.sqrt(np.mean(errors ** 2)) * 100

    # Calculate individual axis errors (Absolute difference)
    x_err = np.mean(np.abs(est_pts[:, 0] - gt_pts[:, 0])) * 100
    y_err = np.mean(np.abs(est_pts[:, 1] - gt_pts[:, 1])) * 100
    z_err = np.mean(np.abs(est_pts[:, 2] - gt_pts[:, 2])) * 100

    return [label, f"{mean_err:.2f} cm", f"{max_err:.2f} cm", f"{rmse:.2f} cm",
            f"{x_err:.2f} cm", f"{y_err:.2f} cm", f"{z_err:.2f} cm"]


def plot_fading_line(ax, x, y, z=None, color='blue', style='-'):
    """Draws a line collection that fades from 10% opacity to 100% opacity"""
    points = np.array([x, y, z]).T.reshape(-1, 1, 3) if z is not None else np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    alphas = np.linspace(0.1, 1.0, len(segments))
    rgba = mcolors.to_rgba(color)
    colors = [(rgba[0], rgba[1], rgba[2], a) for a in alphas]

    if z is not None:
        lc = Line3DCollection(segments, colors=colors, linestyle=style, linewidth=2)
        ax.add_collection3d(lc)
    else:
        lc = LineCollection(segments, colors=colors, linestyle=style, linewidth=2)
        ax.add_collection(lc)


def find_best_alignment(df_uwb, df_gt, label="UWB"):
    """
    Finds the optimal [dx, dy, dz, dt] to align UWB to OptiTrack.
    """
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

    # print("\n" + "=" * 30)
    # print("CALCULATED OPTIMAL PARAMETERS")
    # print("=" * 30)
    # print(f"Copy these into your CONFIGURATION:")
    # print(f"X Offset: {dx:.4f}")
    # print(f"Y Offset: {dy:.4f}")
    # print(f"Z Offset: {dz:.4f}")
    # print(f"TIME_OFFSET = {dt:.4f}")
    # print("=" * 30 + "\n")

    print(f"\nOptimization results for: {label}")
    print("-" * 40)
    print(f"Suggested 'offset': [{dx:.4f}, {dy:.4f}, {dz:.4f}]")
    print(f"Suggested 'time_offset': {dt:.4f}")
    print("-" * 40)

    # We return the values but won't force them into the data automatically
    return dx, dy, dz, dt


def generate_plotly_dashboard(loaded_data, session_folder, report_name):
    """
    Creates an interactive HTML dashboard with 3D, 2D, and time-series plots.
    """
    # Create a 3x2 grid of subplots
    fig = make_subplots(
        rows=3, cols=2,
        specs=[
            [{'type': 'scene'}, {'type': 'xy'}],  # 3D Path | 2D Top Down
            [{'type': 'xy'}, {'type': 'xy'}],  # X Position | Y Position
            [{'type': 'xy'}, {'type': 'xy'}]  # Z Stability | Error
        ],
        subplot_titles=(
            "3D Trajectory Map", "2D Top-Down View",
            "X-Axis Position (m)", "Y-Axis Position (m)",
            "Z-Axis Stability (m)", "Total Error (cm)"
        ),
        vertical_spacing=0.1
    )

    # Find Ground Truth for Error calculation
    gt_key = next(
        (k for k, v in loaded_data.items() if any(d['label'] == k and d.get('is_ground_truth') for d in DATASETS)),
        None)

    global_start_time = min(df['pc_timestamp'].min() for df in loaded_data.values())

    for lbl, df in loaded_data.items():
        time_index = df['pc_timestamp'] - global_start_time
        color = next((d['color'] for d in DATASETS if d['label'] == lbl), 'gray')

        # 1. 3D Trajectory
        fig.add_trace(go.Scatter3d(
            x=df['x'], y=df['y'], z=df['z'],
            mode='lines', name=lbl, line=dict(color=color, width=4),
            hovertemplate="Time: %{text}s<br>X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>",
            text=[f"{t:.2f}" for t in time_index]
        ), row=1, col=1)

        # 2. 2D Top Down
        fig.add_trace(go.Scatter(
            x=df['x'], y=df['y'], mode='lines', name=lbl,
            line=dict(color=color), showlegend=False,
            hovertemplate="X: %{x:.2f}<br>Y: %{y:.2f}<extra></extra>"
        ), row=1, col=2)

        # 3. X Position
        fig.add_trace(
            go.Scatter(x=time_index, y=df['x'], mode='lines', name=lbl, line=dict(color=color), showlegend=False),
            row=2, col=1)

        # 4. Y Position
        fig.add_trace(
            go.Scatter(x=time_index, y=df['y'], mode='lines', name=lbl, line=dict(color=color), showlegend=False),
            row=2, col=2)

        # 5. Z Stability
        fig.add_trace(
            go.Scatter(x=time_index, y=df['z'], mode='lines', name=lbl, line=dict(color=color), showlegend=False),
            row=3, col=1)

        # 6. Error Calculation (relative to GT)
        if gt_key and lbl != gt_key:
            df_gt = loaded_data[gt_key]
            df_sync = pd.merge_asof(
                df.sort_values('pc_timestamp'),
                df_gt.sort_values('pc_timestamp'),
                on='pc_timestamp', suffixes=('_est', '_gt'), direction='nearest'
            )

            rel_time = df_sync['pc_timestamp'] - global_start_time

            # Calculate 3D and individual axis errors in cm
            errs = np.linalg.norm(
                df_sync[['x_est', 'y_est', 'z_est']].values - df_sync[['x_gt', 'y_gt', 'z_gt']].values, axis=1) * 100
            err_x = np.abs(df_sync['x_est'] - df_sync['x_gt']).values * 100
            err_y = np.abs(df_sync['y_est'] - df_sync['y_gt']).values * 100
            err_z = np.abs(df_sync['z_est'] - df_sync['z_gt']).values * 100

            # Total 3D Error (Solid Line)
            fig.add_trace(go.Scatter(
                x=rel_time, y=errs, mode='lines',
                name=f"{lbl} (3D Err)", line=dict(color='purple', width=2),
                hovertemplate="Time: %{x:.2f}s<br>3D Error: %{y:.2f} cm<extra></extra>"
            ), row=3, col=2)

            # X Error (Red Dotted)
            fig.add_trace(go.Scatter(
                x=rel_time, y=err_x, mode='lines',
                name=f"{lbl} (X Err)", line=dict(color='red', dash='dot', width=1.5),
                hovertemplate="Time: %{x:.2f}s<br>X Error: %{y:.2f} cm<extra></extra>"
            ), row=3, col=2)

            # Y Error (Green Dotted)
            fig.add_trace(go.Scatter(
                x=rel_time, y=err_y, mode='lines',
                name=f"{lbl} (Y Err)", line=dict(color='green', dash='dot', width=1.5),
                hovertemplate="Time: %{x:.2f}s<br>Y Error: %{y:.2f} cm<extra></extra>"
            ), row=3, col=2)

            # Z Error (Blue Dotted)
            fig.add_trace(go.Scatter(
                x=rel_time, y=err_z, mode='lines',
                name=f"{lbl} (Z Err)", line=dict(color='deepskyblue', dash='dot', width=1.5),
                hovertemplate="Time: %{x:.2f}s<br>Z Error: %{y:.2f} cm<extra></extra>"
            ), row=3, col=2)

    # Plot Anchors in 3D and 2D
    if SHOW_ANCHORS:
        anchor_csv = os.path.join(session_folder, "[Log]_anchor_positions.csv")

        if os.path.exists(anchor_csv):
            try:
                anchor_df = pd.read_csv(anchor_csv)
                current_anchors = anchor_df[['X', 'Y', 'Z']].values.tolist()
                print(f"Successfully loaded {len(current_anchors)} anchors from CSV.")

                an = np.array(current_anchors)
                fig.add_trace(go.Scatter3d(x=an[:, 0], y=an[:, 1], z=an[:, 2], mode='markers',
                                           marker=dict(symbol='diamond', size=5, color='lime'), name='Anchors'), row=1, col=1)
                fig.add_trace(
                    go.Scatter(x=an[:, 0], y=an[:, 1], mode='markers', marker=dict(symbol='diamond', size=10, color='lime'),
                               name='Anchors', showlegend=False), row=1, col=2)
            except Exception as e:
                print(f"Error reading anchor CSV: {e}")

    fig.update_layout(height=1200, title_text=f"Interactive Report: {report_name}", template="plotly_white")

    # html_path = os.path.join(session_folder, f"Interactive_Report_{report_name.replace(' ', '_')}.html")
    # safe_name = "[Report]_Interactive_Report.html"
    safe_name = f"[Report]_{REPORT_INFO['name'].replace(' ', '_')}.html"
    html_path = os.path.join(session_folder, safe_name)
    fig.write_html(html_path)
    print(f" -> Interactive Plotly report saved: {html_path}")


def run_dashboard(session_folder=None, skip_popup=False):
    global REPORT_INFO, SHOW_ANCHORS, SAVE_PDF, SAVE_INDIVIDUAL_PNGS

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
                SAVE_PDF = config.get('save_pdf', SAVE_PDF)
                SAVE_INDIVIDUAL_PNGS = config.get('save_pngs', SAVE_INDIVIDUAL_PNGS)
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
            SAVE_PDF = dialog.result['save_pdf']
            SAVE_INDIVIDUAL_PNGS = dialog.result['save_pngs']

    # Setup Figure (A4-ish proportions: 8.5 x 11)
    fig_unused = plt.figure(figsize=(14, 18))
    fig = plt.figure(figsize=(14, 18))
    gs = fig.add_gridspec(5, 2, height_ratios=[0.15, 1, 1, 1, 0.25])

    # Header / Metadata
    ax_header = fig.add_subplot(gs[0, :])
    ax_header.axis('off')
    header_text = f"TEST REPORT: {REPORT_INFO['name']}\n"
    header_text += f"Exported: {datetime.now().strftime('%Y-%m-%d %H:%M')}\n"
    header_text += f"Notes: {REPORT_INFO['notes']}"
    ax_header.text(0, 0.5, header_text, fontsize=14, weight='bold', verticalalignment='center')

    # ax1 = fig.add_subplot(gs[1, 0], projection='3d')
    # ax2 = fig.add_subplot(gs[1, 1])
    # ax3 = fig.add_subplot(gs[2, 0])
    # ax4 = fig.add_subplot(gs[2, 1])
    # ax5 = fig.add_subplot(gs[3, :])  # Error over time spans full width
    # ax_table = fig.add_subplot(gs[4, :])
    # ax_table.axis('off')

    ax1 = fig.add_subplot(gs[1, 0], projection='3d')
    ax2 = fig.add_subplot(gs[1, 1])
    ax_x = fig.add_subplot(gs[2, 0])
    ax_y = fig.add_subplot(gs[2, 1])
    ax3 = fig.add_subplot(gs[3, 0])
    ax4 = fig_unused.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[3, 1])  # Error over time spans full width
    ax_table = fig.add_subplot(gs[4, :])
    ax_table.axis('off')

    fig.canvas.manager.set_window_title('Drone Localization Dashboard')

    # 1. Plot Anchors
    if SHOW_ANCHORS:
        anchor_csv = os.path.join(session_folder, "[Log]_anchor_positions.csv")

        if os.path.exists(anchor_csv):
            try:
                anchor_df = pd.read_csv(anchor_csv)
                current_anchors = anchor_df[['X', 'Y', 'Z']].values.tolist()
                print(f"Successfully loaded {len(current_anchors)} anchors from CSV.")
                anchors_arr = np.array(current_anchors)
                # 3D Map
                ax1.scatter(anchors_arr[:, 0], anchors_arr[:, 1], anchors_arr[:, 2],
                            c='lime', marker='^', s=150, label='UWB Anchors',
                            edgecolors='black', zorder=10)
                # 2D Top-Down
                ax2.scatter(anchors_arr[:, 0], anchors_arr[:, 1],
                            c='lime', marker='^', s=150, label='UWB Anchors',
                            edgecolors='black', zorder=5)
            except Exception as e:
                print(f"Error reading anchor CSV: {e}")
        # else:
        #     # Fallback to the global ANCHORS list from your CONFIGURATION
        #     current_anchors = ANCHORS
        #     print("No CSV found, using manual ANCHORS from config.")

    loaded_data = {}
    gt_key = None
    table_data = []

    # 2. Load Data First
    for ds in DATASETS:
        # AUTOMATICALLY find the latest file based on the prefix
        filepath = get_latest_file(ds['prefix'], session_folder)
        if not filepath:
            print(f"WARNING: No file found for {ds['prefix']}")
            continue

        df = load_position_data(filepath)
        if df is None or df.empty: continue

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
        loaded_data[lbl] = df
        if ds.get('is_ground_truth'): gt_key = lbl

    if not loaded_data:
        print("No data loaded. Exiting.")
        return

    # --- FIND GLOBAL START TIME ---
    global_start_time = min(df['pc_timestamp'].min() for df in loaded_data.values())

    # 2.5 Plot Traces using Global Time
    for ds in DATASETS:
        lbl = ds['label']
        if lbl not in loaded_data: continue

        df = loaded_data[lbl]
        col, sty = ds['color'], ds['style']

        # --- TIME NORMALIZATION FIX ---
        time_index = df['pc_timestamp'] - global_start_time

        # --- 3D & 2D Standard Paths ---
        ax1.plot(df['x'], df['y'], df['z'], label=lbl, color=col, linestyle=sty, linewidth=2)
        ax2.plot(df['x'], df['y'], label=lbl, color=col, linestyle=sty, linewidth=2)

        # Draw faint dots to set axis bounds and show sample rate
        ax1.scatter(df['x'], df['y'], df['z'], color=col, s=5, alpha=0.1)
        ax2.scatter(df['x'], df['y'], color=col, s=5, alpha=0.1)

        # --- Time-series Graphs ---
        ax3.plot(time_index, df['z'], label=lbl, color=col, linestyle=sty, linewidth=2)
        ax_y.plot(time_index, df['y'], label=lbl, color=col, linestyle=sty, linewidth=2)
        ax_x.plot(time_index, df['x'], label=lbl, color=col, linestyle=sty, linewidth=2)
        speeds = calculate_velocity(df)
        ax4.plot(time_index, speeds, label=lbl, color=col, linestyle=sty, linewidth=1.5, alpha=0.8)

    if gt_key and "UWB Raw" in loaded_data:
        # This just prints the suggestions to the console
        find_best_alignment(loaded_data["UWB Raw"], loaded_data[gt_key])

    # 3. Calculate Errors and Build Table (Time-Synchronized)
    if gt_key and gt_key in loaded_data:
        df_gt = loaded_data[gt_key]
        # Normalize Ground Truth time for the separate plots
        gt_time = df_gt['pc_timestamp'] - df_gt['pc_timestamp'].iloc[0]

        for lbl, df in loaded_data.items():
            if lbl == gt_key: continue

            # Sync for Error Calculation
            df_sync = pd.merge_asof(
                df.sort_values('pc_timestamp'),
                df_gt.sort_values('pc_timestamp'),
                on='pc_timestamp',
                suffixes=('_est', '_gt'),
                direction='nearest'
            )

            # Calculate total Euclidean error for the main dashboard (ax5)
            errs = np.linalg.norm(
                df_sync[['x_est', 'y_est', 'z_est']].values -
                df_sync[['x_gt', 'y_gt', 'z_gt']].values,
                axis=1
            )
            rel_time = df_sync['pc_timestamp'] - global_start_time
            ax5.plot(rel_time, errs * 100, label=f"Error: {lbl}", color='purple', linewidth=1.5)

            # Calculate individual axis errors
            err_x = np.abs(df_sync['x_est'] - df_sync['x_gt']).values
            err_y = np.abs(df_sync['y_est'] - df_sync['y_gt']).values
            err_z = np.abs(df_sync['z_est'] - df_sync['z_gt']).values

            # table_data.append([lbl, f"{np.mean(errs) * 100:.2f} cm", f"{np.max(errs) * 100:.2f} cm",
            #                    f"{np.sqrt(np.mean(errs ** 2)) * 100:.2f} cm"])

            table_data.append([
                lbl,
                f"{np.mean(errs) * 100:.1f} cm",
                f"{np.max(errs) * 100:.1f} cm",
                f"{np.sqrt(np.mean(errs ** 2)) * 100:.1f} cm",
                f"{np.mean(err_x) * 100:.1f} cm",
                f"{np.mean(err_y) * 100:.1f} cm",
                f"{np.mean(err_z) * 100:.1f} cm"
            ])

    # if table_data:
    #     columns = ['Sensor Data', 'Mean Error (cm)', 'Max Error (cm)', 'RMSE (Official)']
    #     table = ax_table.table(cellText=table_data, colLabels=columns, loc='center', cellLoc='center')
    #     table.auto_set_font_size(False)
    #     table.set_fontsize(12)
    #     table.scale(1, 1.8)  # Stretch cells slightly for readability

    if table_data:
        columns = ['Sensor', 'Mean (3D)', 'Max (3D)', 'RMSE (3D)', 'Err X', 'Err Y', 'Err Z']
        table = ax_table.table(cellText=table_data, colLabels=columns, loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)  # Reduced from 12 to fit the extra columns
        table.scale(1, 1.8)

    # 4. Graph Formatting
    ax1.set_title('3D Trajectory Map')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_zlim([0, 4])
    ax1.legend()

    ax2.set_title('2D Top-Down View')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.axis('equal')
    ax2.grid(True, linestyle=':')
    ax2.legend()

    ax3.set_title('Z-Axis (Altitude) Stability Profile')
    ax3.set_xlabel('Measurement Index (Time)')
    ax3.set_ylabel('Height (m)')
    ax3.set_ylim([0, 4])
    ax3.grid(True, linestyle=':')
    ax3.legend()

    ax4.set_title('Velocity/Noise Profile (Spike = UWB Jitter)')
    ax4.set_xlabel('Measurement Index (Time)')
    ax4.set_ylabel('Speed (m/s)')
    ax4.grid(True, linestyle=':')
    ax4.legend()

    ax5.set_title('Error (Difference between UWB and ground Truth)')
    ax5.set_xlabel('Measurement Index (Time)')
    ax5.set_ylabel('Error (cm)')
    ax5.grid(True, linestyle=':')
    ax5.legend()

    ax_x.set_title('X-Axis Position Comparison')
    ax_x.set_xlabel('Elapsed Time (s)')
    ax_x.set_ylabel('X Position (m)')
    ax_x.grid(True, linestyle=':')
    ax_x.legend()

    ax_y.set_title('Y-Axis Position Comparison')
    ax_y.set_xlabel('Elapsed Time (s)')
    ax_y.set_ylabel('Y Position (m)')
    ax_y.grid(True, linestyle=':')
    ax_y.legend()

    plt.tight_layout()

    if SAVE_PDF:
        report_title = f"[Report]_{REPORT_INFO['name'].replace(' ', '_')}"
        pdf_filename = os.path.join(session_folder, f"{report_title}.pdf")
        plt.savefig(pdf_filename, dpi=300, bbox_inches='tight')
        print(f"Report exported successfully to: {pdf_filename}")

    if SAVE_INDIVIDUAL_PNGS:
        export_map = {
            '3d_map': (ax1, "[Plot]_3D_Map"),
            '2d_top_down': (ax2, "[Plot]_2D_View"),
            'x_axis_comparison': (ax_x, "[Plot]_X_Comparison"),
            'y_axis_comparison': (ax_y, "[Plot]_Y_Comparison"),
            'z_stability': (ax3, "[Plot]_Z_Altitude"),
            'velocity': (ax4, "[Plot]_Velocity"),
            'total_error': (ax5, "[Plot]_Total_Error")
        }

        for key, (axis, filename) in export_map.items():
            if EXPORT_LIST.get(key):
                png_path = os.path.join(session_folder, f"{filename}.png")
                extent = axis.get_tightbbox(fig.canvas.get_renderer()).transformed(fig.dpi_scale_trans.inverted())
                fig.savefig(png_path, bbox_inches=extent, dpi=300)
                print(f" -> Saved: {filename}.png")

    # if loaded_data:
    generate_plotly_dashboard(loaded_data, session_folder, REPORT_INFO['name'])

    plt.show()

    # 1. Get the parent directory (e.g., 'measurements/')
    parent_dir = os.path.dirname(session_folder)

    # 2. Construct the new path
    new_folder_path = os.path.join(parent_dir, report_title)

    # 3. Rename the folder safely
    try:
        os.rename(session_folder, new_folder_path)
        print(f"Folder successfully renamed to: {report_title}")
    except PermissionError:
        print("Could not rename folder. Make sure no CSV files are open in Excel!")

if __name__ == "__main__":
    run_dashboard()




#OLD
# 3. Define your Anchor coordinates [X, Y, Z] in meters
# ANCHORS = [
#     # #Tripod Top Anchors
#     # [-1.9,-1.75,1.17], [-1.86,1.35,1.19],
#     # [2.26,1.34,1.19], [2.25,-1.75,1.18],
#     #
#     # #Bottom Anchors
#     # [-1.9,-1.75,0.07], [-1.86,1.35,0.07],
#     # [2.26,1.34,0.07], [2.25,-1.75,0.07]
#
#     # #Wall anchors:
#     # [0.00, 0.00, 2.48], [2.96, 0.00, 2.48],
#     # [5.8, 0.1, 2.40], [8.03, 0.1, 2.38],
#     # [7.36, 6.53, 2.22], [5.88, 6.92, 2.59],
#     # [2.39, 6.8, 2.65], [0.54, 6.99, 2.56]
#
#     # 4 middle wall anchors measured with red pole (kind of accurate but not really) (2,3,6,7)
#     # And offset not applied yet
#     [0.790,3.542,2.430], [-1.740,3.533,2.390],
#     [-1.330,-3.532,2.475], [1.590,-3.521,2.510],
#
# ]