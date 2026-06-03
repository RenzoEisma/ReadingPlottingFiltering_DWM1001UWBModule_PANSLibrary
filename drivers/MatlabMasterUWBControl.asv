% =========================================================================
% MATLAB MASTER UWB CONTROL
% Author: Renzo Eisma / rewritten with ChatGPT
% Date: 06/2026
%
% Purpose:
%   Clean MATLAB coordinator for the current UWB-only structure.
%
% Current version supports:
%   - Python UDP UWB listener input on port 5005
%   - Python UDP OptiTrack input on port 5006
%   - Python settings packet on port 5004
%   - GeneralFilter.m for UWB position filtering
%   - final_position selection inside MATLAB
%   - simple useful logging
%
% Not included yet:
%   - IMU fusion
%   - robot control
%   - ROS publishing
%   - custom PCB ROS input
%   - GPS RTK ROS input
%
% Data flow:
%   Python UWB UDP -> uwb_sample -> GeneralFilter -> uwb_general_filtered
%                                                        |
%   Python Opti UDP -> opti_sample -----------------------|-> final_position
% =========================================================================

function MatlabMasterUWBControl()

%% 0. CLEANUP
% =========================================================================
clear classes;
close all;
clc;

try
    rosshutdown;
catch
end
pause(0.2);

fprintf('\n============================================================\n');
fprintf('[MASTER] MATLAB Master UWB Control started.\n');
fprintf('============================================================\n\n');

%% 1. MANUAL MATLAB CONFIGURATION
% =========================================================================
% Keep these settings here so the final position source is controlled in
% MATLAB and not from Python.

% Options: "UWB_GENERAL" or "OPTITRACK"
FINAL_POSITION_SOURCE = "UWB_GENERAL";

% This version only supports Python UDP UWB input.
UWB_SOURCE_MATLAB = "PYTHON_UDP";

% Runtime settings
T_exp = 2400;                    % maximum runtime [s]
T_final_log = 1/30;              % final_position logging rate [s]
MAIN_LOOP_PAUSE = 0.001;         % small pause to avoid maxing CPU

% Startup sensor wait
WAIT_FOR_REQUIRED_SENSORS = true;
SENSOR_WAIT_TIMEOUT = 30;        % seconds
SENSOR_FRESH_TIMEOUT = 2.0;      % seconds

% GeneralFilter settings
FILTER_DEFAULT_DT = 0.1;
ENABLE_GENERAL_FILTER_LOG = true;

% Simple status printing
PRINT_EVERY_UWB_PACKETS = 10;
PRINT_EVERY_OPTI_PACKETS = 200;
PRINT_EVERY_FINAL_LINES = 30;

%% 2. UDP PORT SETUP
% =========================================================================
SETTINGS_PORT = 5004;
UWB_PORT      = 5005;
OPTI_PORT     = 5006;

fprintf('[MASTER] Opening UDP ports...\n');
try
    u_settings = udpport("LocalHost", "127.0.0.1", "LocalPort", SETTINGS_PORT);
    u_uwb      = udpport("LocalHost", "127.0.0.1", "LocalPort", UWB_PORT);
    u_opti     = udpport("LocalHost", "127.0.0.1", "LocalPort", OPTI_PORT);
catch ME
    fprintf('[MASTER] Failed to open UDP ports: %s\n', ME.message);
    fprintf('[MASTER] Make sure no old MATLAB process is still using ports %d, %d or %d.\n', ...
        SETTINGS_PORT, UWB_PORT, OPTI_PORT);
    return;
end

fprintf('[MASTER] Settings UDP: 127.0.0.1:%d\n', SETTINGS_PORT);
fprintf('[MASTER] UWB UDP:      127.0.0.1:%d\n', UWB_PORT);
fprintf('[MASTER] Opti UDP:     127.0.0.1:%d\n\n', OPTI_PORT);

%% 3. WAIT FOR SETTINGS PACKET FROM PYTHON
% =========================================================================
% Python still owns session creation and sends the session folder/name.
% MATLAB uses that information for logging.

disp('[MASTER] Waiting for settings packet on port 5004...');
settings = struct();
settings_received = false;

while ~settings_received
    packet = readLatestUdpPacket(u_settings);

    if strlength(packet) > 0
        [settings, settings_received] = parseSettingsPacket(packet);

        if settings_received
            disp('[MASTER] Settings packet received.');
        else
            disp('[MASTER] Received invalid settings packet. Waiting for a new one...');
        end
    end

    pause(0.05);
end

%% 4. APPLY SETTINGS FROM PYTHON, BUT KEEP FINAL POSITION SOURCE IN MATLAB
% =========================================================================
uwb_settings = getFieldDefault(settings, 'uwb', struct());
gt_settings  = getFieldDefault(settings, 'ground_truth', struct());

session_name = string(getFieldDefault(settings, 'session_name', "Session_Unknown"));
session_dir  = string(getFieldDefault(settings, 'session_dir', pwd));

ENABLE_UWB = toLogical(getFieldDefault(uwb_settings, 'enabled', false));
PYTHON_UWB_SOURCE = string(getFieldDefault(uwb_settings, 'source', "Listener"));
UWB_READ_TYPE = string(getFieldDefault(uwb_settings, 'read_type', "Tag Position"));
UWB_NETWORK_SCALE = string(getFieldDefault(uwb_settings, 'network_scale', "1 Network / 1 Listener"));

ENABLE_GT = toLogical(getFieldDefault(gt_settings, 'enabled', false));
GT_SOURCE = string(getFieldDefault(gt_settings, 'type', "OptiTrack"));

if ~isfolder(session_dir)
    mkdir(session_dir);
end

% This script only supports Python listener UWB for now.
if ENABLE_UWB && PYTHON_UWB_SOURCE ~= "Listener"
    fprintf('[MASTER] Warning: Python selected UWB source "%s".\n', PYTHON_UWB_SOURCE);
    fprintf('[MASTER] This simplified MATLAB script only supports Python listener UDP for now. UWB disabled.\n');
    ENABLE_UWB = false;
end

% This script only supports OptiTrack UDP as ground truth for now.
if ENABLE_GT && GT_SOURCE ~= "OptiTrack"
    fprintf('[MASTER] Warning: Python selected ground truth "%s".\n', GT_SOURCE);
    fprintf('[MASTER] This simplified MATLAB script only supports OptiTrack UDP for now. Ground truth disabled.\n');
    ENABLE_GT = false;
end

fprintf('\n[MASTER] Session name: %s\n', session_name);
fprintf('[MASTER] Session folder: %s\n', session_dir);
fprintf('[MASTER] MATLAB UWB source: %s\n', UWB_SOURCE_MATLAB);
fprintf('[MASTER] Python UWB enabled: %d | Read mode: %s | Network scale: %s\n', ...
    ENABLE_UWB, UWB_READ_TYPE, UWB_NETWORK_SCALE);
fprintf('[MASTER] OptiTrack enabled: %d\n', ENABLE_GT);
fprintf('[MASTER] Final position source: %s\n\n', FINAL_POSITION_SOURCE);

%% 5. FILTER AND LOGGING SETUP
% =========================================================================
if ENABLE_GENERAL_FILTER_LOG
    general_filter_log = fullfile(session_dir, "[Log]_uwb_general_filter_" + session_name + ".csv");
else
    general_filter_log = '';
end

filter_config = struct();
filter_config.LOG_TO_CSV = ENABLE_GENERAL_FILTER_LOG;

uwb_general_filter = GeneralFilter(FILTER_DEFAULT_DT, general_filter_log);
uwb_general_filter.configure(filter_config);

if ENABLE_GENERAL_FILTER_LOG
    fprintf('[MASTER] GeneralFilter log: %s\n', general_filter_log);
end

final_log_path = fullfile(session_dir, "[Log]_final_position_matlab_" + session_name + ".csv");
final_log_fid = openFinalPositionLog(final_log_path);

if final_log_fid > 0
    fprintf('[MASTER] Final position log: %s\n\n', final_log_path);
else
    fprintf('[MASTER] Warning: final_position logging disabled.\n\n');
end

%% 6. STATE VARIABLES
% =========================================================================
t_exp = tic;
t_final_log = tic;

latest_uwb_sample = makeEmptyUwbSample();
latest_uwb_general = makeEmptyGeneralFilteredSample();
latest_opti_sample = makeEmptyOptiSample();
final_position = makeEmptyFinalPosition();

last_uwb_receive_time = -Inf;
last_opti_receive_time = -Inf;

print_counter_uwb = 0;
print_counter_opti = 0;
print_counter_final = 0;

require_uwb = ENABLE_UWB && FINAL_POSITION_SOURCE == "UWB_GENERAL";
require_opti = ENABLE_GT && FINAL_POSITION_SOURCE == "OPTITRACK";

%% 7. WAIT FOR REQUIRED SENSORS
% =========================================================================
if WAIT_FOR_REQUIRED_SENSORS
    disp('[MASTER] Waiting for required sensors to become active...');
    sensor_wait_start = tic;

    while toc(sensor_wait_start) < SENSOR_WAIT_TIMEOUT
        [latest_uwb_sample, latest_uwb_general, last_uwb_receive_time, print_counter_uwb] = ...
            receiveAndFilterUwb(u_uwb, uwb_general_filter, ENABLE_UWB, t_exp, ...
            latest_uwb_sample, latest_uwb_general, last_uwb_receive_time, print_counter_uwb, PRINT_EVERY_UWB_PACKETS);

        [latest_opti_sample, last_opti_receive_time, print_counter_opti] = ...
            receiveOptiTrack(u_opti, ENABLE_GT, t_exp, latest_opti_sample, ...
            last_opti_receive_time, print_counter_opti, PRINT_EVERY_OPTI_PACKETS);

        current_time = toc(t_exp);
        uwb_ready = ~require_uwb || isSensorFresh(last_uwb_receive_time, current_time, SENSOR_FRESH_TIMEOUT);
        opti_ready = ~require_opti || isSensorFresh(last_opti_receive_time, current_time, SENSOR_FRESH_TIMEOUT);

        if uwb_ready && opti_ready
            disp('[MASTER] Required sensors are active.');
            break;
        end

        if require_uwb && ~uwb_ready
            fprintf('[MASTER] Waiting for UWB listener data on port %d...\n', UWB_PORT);
        end
        if require_opti && ~opti_ready
            fprintf('[MASTER] Waiting for OptiTrack data on port %d...\n', OPTI_PORT);
        end

        pause(0.5);
    end

    current_time = toc(t_exp);
    uwb_ready = ~require_uwb || isSensorFresh(last_uwb_receive_time, current_time, SENSOR_FRESH_TIMEOUT);
    opti_ready = ~require_opti || isSensorFresh(last_opti_receive_time, current_time, SENSOR_FRESH_TIMEOUT);

    if ~(uwb_ready && opti_ready)
        disp('[MASTER] Warning: not all required sensors became active before timeout.');
        disp('[MASTER] Script will continue for logging/debugging.');
    else
        disp('[MASTER] Sensor startup check complete.');
    end
end

%% 8. MAIN LOOP
% =========================================================================
disp('[MASTER] Entering main loop. Press Ctrl+C to stop.');

try
    while toc(t_exp) < T_exp

        % -------------------------------------------------------------
        % A. UWB from Python UDP -> GeneralFilter
        % -------------------------------------------------------------
        [latest_uwb_sample, latest_uwb_general, last_uwb_receive_time, print_counter_uwb] = ...
            receiveAndFilterUwb(u_uwb, uwb_general_filter, ENABLE_UWB, t_exp, ...
            latest_uwb_sample, latest_uwb_general, last_uwb_receive_time, print_counter_uwb, PRINT_EVERY_UWB_PACKETS);

        % -------------------------------------------------------------
        % B. OptiTrack from Python UDP
        % -------------------------------------------------------------
        [latest_opti_sample, last_opti_receive_time, print_counter_opti] = ...
            receiveOptiTrack(u_opti, ENABLE_GT, t_exp, latest_opti_sample, ...
            last_opti_receive_time, print_counter_opti, PRINT_EVERY_OPTI_PACKETS);

        % -------------------------------------------------------------
        % C. Select final_position inside MATLAB
        % -------------------------------------------------------------
        final_position = selectFinalPosition(FINAL_POSITION_SOURCE, latest_uwb_general, latest_opti_sample);

        % -------------------------------------------------------------
        % D. Log final_position at fixed rate
        % -------------------------------------------------------------
        if toc(t_final_log) >= T_final_log
            t_final_log = tic;
            logFinalPosition(final_log_fid, final_position, latest_uwb_general, latest_opti_sample);

            print_counter_final = print_counter_final + 1;
            if mod(print_counter_final, PRINT_EVERY_FINAL_LINES) == 0 && final_position.valid
                fprintf('[FINAL] Source: %s -> X: %.3f, Y: %.3f, Z: %.3f\n', ...
                    final_position.source, final_position.position(1), final_position.position(2), final_position.position(3));
            end
        end

        drawnow;
        pause(MAIN_LOOP_PAUSE);
    end

catch ME
    fprintf('[MASTER] CRASH: %s\n', ME.message);
    fprintf('[MASTER] File: %s | Line: %d\n', ME.stack(1).file, ME.stack(1).line);
end

%% 9. SAFE SHUTDOWN
% =========================================================================
disp('[MASTER] Cleaning up resources...');

try
    uwb_general_filter.closeLog();
catch
end

closeFileSafe(final_log_fid);

try
    clear u_settings u_uwb u_opti;
catch
end

try
    rosshutdown;
catch
end

disp('[MASTER] Shutdown complete.');

end

%% ========================================================================
% LOCAL HELPER FUNCTIONS
% ========================================================================

% -------------------------------------------------------------------------
function packet = readLatestUdpPacket(u)
packet = "";
try
    if u.NumBytesAvailable <= 0
        return;
    end

    raw_data = read(u, u.NumBytesAvailable, "string");

    if isstring(raw_data)
        if numel(raw_data) > 1
            packet = raw_data(end);
        else
            packet = raw_data;
        end
    elseif ischar(raw_data)
        packet = string(raw_data);
    else
        packet = string(char(raw_data));
    end

    packet = strtrim(packet);

    % If multiple lines are received, use the latest non-empty one.
    if contains(packet, newline)
        parts = splitlines(packet);
        parts = parts(strlength(strtrim(parts)) > 0);
        if ~isempty(parts)
            packet = strtrim(parts(end));
        end
    end

catch ME
    fprintf('[UDP] Read error: %s\n', ME.message);
    packet = "";
end
end

% -------------------------------------------------------------------------
function [settings, valid] = parseSettingsPacket(packet)
settings = struct();
valid = false;

try
    decoded = jsondecode(char(packet));
    if isfield(decoded, 'packet_type')
        if string(decoded.packet_type) == "settings"
            settings = decoded;
            valid = true;
        end
    end
catch ME
    fprintf('[MASTER] Settings JSON parse error: %s\n', ME.message);
end
end

% -------------------------------------------------------------------------
function value = getFieldDefault(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name)
    value = s.(field_name);
    if isempty(value)
        value = default_value;
    end
else
    value = default_value;
end
end

% -------------------------------------------------------------------------
function value = toLogical(input_value)
if islogical(input_value)
    value = input_value;
elseif isnumeric(input_value)
    value = input_value ~= 0;
elseif ischar(input_value) || isstring(input_value)
    text_value = lower(strtrim(string(input_value)));
    value = any(text_value == ["true", "1", "yes", "on"]);
else
    value = false;
end
end

% -------------------------------------------------------------------------
% UWB packet format from Python:
% timestamp,x,y,z,quality,listener_id,network_id,position_type
function uwb = parseUwbPacket(packet)
uwb = makeEmptyUwbSample();

try
    parts = strsplit(char(strtrim(packet)), ',');
    if numel(parts) < 8
        return;
    end

    timestamp = str2double(parts{1});
    x = str2double(parts{2});
    y = str2double(parts{3});
    z = str2double(parts{4});
    quality = str2double(parts{5});
    listener_id = str2double(parts{6});
    network_id = str2double(parts{7});
    position_type = string(strtrim(parts{8}));

    if any(isnan([timestamp, x, y, z]))
        return;
    end

    uwb.position = [x, y, z];
    uwb.quality = quality;
    uwb.timestamp = timestamp;
    uwb.valid = true;
    uwb.source = "python_udp";
    uwb.listener_id = listener_id;
    uwb.network_id = network_id;
    uwb.position_type = position_type;

catch
    uwb.valid = false;
end
end

% -------------------------------------------------------------------------
% OptiTrack packet format from Python:
% timestamp,x,y,z,quality,rigid_body_id,source_type
function opti = parseOptiPacket(packet)
opti = makeEmptyOptiSample();

try
    parts = strsplit(char(strtrim(packet)), ',');
    if numel(parts) < 7
        return;
    end

    timestamp = str2double(parts{1});
    x = str2double(parts{2});
    y = str2double(parts{3});
    z = str2double(parts{4});
    quality = str2double(parts{5});
    rigid_body_id = str2double(parts{6});
    source_type = string(strtrim(parts{7}));

    if any(isnan([timestamp, x, y, z]))
        return;
    end

    opti.position = [x, y, z];
    opti.velocity = [NaN, NaN, NaN];
    opti.quality = quality;
    opti.timestamp = timestamp;
    opti.valid = true;
    opti.source = "optitrack";
    opti.rigid_body_id = rigid_body_id;
    opti.source_type = source_type;

catch
    opti.valid = false;
end
end

% -------------------------------------------------------------------------
function [latest_uwb_sample, latest_uwb_general, last_receive_time, print_counter] = ...
    receiveAndFilterUwb(u_uwb, general_filter, enable_uwb, t_exp, ...
    latest_uwb_sample, latest_uwb_general, last_receive_time, print_counter, print_every)

if ~enable_uwb
    return;
end

packet = readLatestUdpPacket(u_uwb);
if strlength(packet) == 0
    return;
end

uwb_sample = parseUwbPacket(packet);
if ~uwb_sample.valid
    fprintf('[MASTER] Invalid UWB packet: %s\n', packet);
    return;
end

latest_uwb_sample = uwb_sample;
last_receive_time = toc(t_exp);

latest_uwb_general = general_filter.processUwbSample(uwb_sample);

print_counter = print_counter + 1;
if mod(print_counter, print_every) == 0
    fprintf('[UWB] Raw [%.2f %.2f %.2f] -> General [%.2f %.2f %.2f] | accepted=%d | reason=%s\n', ...
        uwb_sample.position(1), uwb_sample.position(2), uwb_sample.position(3), ...
        latest_uwb_general.position(1), latest_uwb_general.position(2), latest_uwb_general.position(3), ...
        latest_uwb_general.accepted, char(string(latest_uwb_general.rejection_reason)));
end
end

% -------------------------------------------------------------------------
function [latest_opti_sample, last_receive_time, print_counter] = ...
    receiveOptiTrack(u_opti, enable_gt, t_exp, latest_opti_sample, last_receive_time, print_counter, print_every)

if ~enable_gt
    return;
end

packet = readLatestUdpPacket(u_opti);
if strlength(packet) == 0
    return;
end

opti_sample = parseOptiPacket(packet);
if ~opti_sample.valid
    fprintf('[MASTER] Invalid OptiTrack packet: %s\n', packet);
    return;
end

latest_opti_sample = opti_sample;
last_receive_time = toc(t_exp);

print_counter = print_counter + 1;
if mod(print_counter, print_every) == 0
    fprintf('[OPTI] X: %.3f, Y: %.3f, Z: %.3f | quality=%.2f\n', ...
        opti_sample.position(1), opti_sample.position(2), opti_sample.position(3), opti_sample.quality);
end
end

% -------------------------------------------------------------------------
function final_position = selectFinalPosition(final_source, uwb_general, opti_sample)
final_position = makeEmptyFinalPosition();

switch upper(string(final_source))
    case "UWB_GENERAL"
        if uwb_general.valid
            final_position.position = uwb_general.position;
            final_position.velocity = uwb_general.velocity;
            final_position.timestamp = uwb_general.timestamp;
            final_position.valid = true;
            final_position.source = "uwb_general";
            final_position.quality = uwb_general.quality;
        end

    case "OPTITRACK"
        if opti_sample.valid
            final_position.position = opti_sample.position;
            final_position.velocity = opti_sample.velocity;
            final_position.timestamp = opti_sample.timestamp;
            final_position.valid = true;
            final_position.source = "optitrack";
            final_position.quality = opti_sample.quality;
        end

    otherwise
        final_position.valid = false;
        final_position.source = "invalid_final_source";
end
end

% -------------------------------------------------------------------------
function fid = openFinalPositionLog(log_path)
fid = -1;
try
    [folder, ~, ~] = fileparts(log_path);
    if ~isempty(folder) && ~exist(folder, 'dir')
        mkdir(folder);
    end

    fid = fopen(log_path, 'w');
    if fid > 0
        fprintf(fid, ['timestamp,final_source,final_x,final_y,final_z,final_vx,final_vy,final_vz,', ...
            'final_valid,final_quality,uwb_general_x,uwb_general_y,uwb_general_z,', ...
            'uwb_accepted,uwb_rejection_reason,opti_x,opti_y,opti_z,opti_valid\n']);
    end
catch ME
    fprintf('[MASTER] Could not open final position log: %s\n', ME.message);
    fid = -1;
end
end

% -------------------------------------------------------------------------
function logFinalPosition(fid, final_position, uwb_general, opti_sample)
if fid <= 0
    return;
end

fprintf(fid, '%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%d,%s,%.6f,%.6f,%.6f,%d\n', ...
    final_position.timestamp, ...
    safeCsvText(final_position.source), ...
    final_position.position(1), final_position.position(2), final_position.position(3), ...
    final_position.velocity(1), final_position.velocity(2), final_position.velocity(3), ...
    final_position.valid, ...
    final_position.quality, ...
    uwb_general.position(1), uwb_general.position(2), uwb_general.position(3), ...
    getStructFieldDefault(uwb_general, 'accepted', false), ...
    safeCsvText(getStructFieldDefault(uwb_general, 'rejection_reason', 'none')), ...
    opti_sample.position(1), opti_sample.position(2), opti_sample.position(3), ...
    opti_sample.valid);
end

% -------------------------------------------------------------------------
function is_fresh = isSensorFresh(last_receive_time, current_time, freshness_timeout)
is_fresh = isfinite(last_receive_time) && ((current_time - last_receive_time) <= freshness_timeout);
end

% -------------------------------------------------------------------------
function uwb = makeEmptyUwbSample()
uwb.position = [NaN, NaN, NaN];
uwb.quality = NaN;
uwb.timestamp = NaN;
uwb.valid = false;
uwb.source = "python_udp";
uwb.listener_id = NaN;
uwb.network_id = NaN;
uwb.position_type = "unknown";
end

% -------------------------------------------------------------------------
function out = makeEmptyGeneralFilteredSample()
out.position = [NaN, NaN, NaN];
out.velocity = [NaN, NaN, NaN];
out.timestamp = NaN;
out.quality = NaN;
out.valid = false;
out.source = "general_filter";
out.raw_position = [NaN, NaN, NaN];
out.input_source = "none";
out.accepted = false;
out.rejection_reason = "not_initialized";
out.debug = struct();
end

% -------------------------------------------------------------------------
function opti = makeEmptyOptiSample()
opti.position = [NaN, NaN, NaN];
opti.velocity = [NaN, NaN, NaN];
opti.quality = NaN;
opti.timestamp = NaN;
opti.valid = false;
opti.source = "optitrack";
opti.rigid_body_id = NaN;
opti.source_type = "unknown";
end

% -------------------------------------------------------------------------
function final_position = makeEmptyFinalPosition()
final_position.position = [NaN, NaN, NaN];
final_position.velocity = [NaN, NaN, NaN];
final_position.timestamp = NaN;
final_position.valid = false;
final_position.source = "none";
final_position.quality = NaN;
end

% -------------------------------------------------------------------------
function value = getStructFieldDefault(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name)
    value = s.(field_name);
else
    value = default_value;
end
end

% -------------------------------------------------------------------------
function txt = safeCsvText(value)
txt = char(string(value));
txt = strrep(txt, ',', '_');
txt = strrep(txt, newline, ' ');
end

% -------------------------------------------------------------------------
function closeFileSafe(fid)
try
    if fid > 0
        fclose(fid);
    end
catch
end
end
