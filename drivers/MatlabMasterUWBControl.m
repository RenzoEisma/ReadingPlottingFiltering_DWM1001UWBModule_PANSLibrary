% =========================================================================
% MATLAB MASTER UWB CONTROL
% =========================================================================
% Author: Renzo Eisma
% Date: 06/2026
% Assistance note:
% ChatGPT Pro 5.5 Thinking Extended was used to clean up variable names,
% comments, line spacing, and error logging in the terminal. The rewritten
% version was checked by a human. The original concept, code logic, and 
% project structure were created by Renzo Eisma.
%
% Purpose:
% Central MATLAB coordinator for the UWB localization/control framework.
% Filtering, sensor reading, and robot control are handled by separate
% scripts/classes.
%
% Included:
% - Python UDP UWB listener input on port 5005
% - Python UDP OptiTrack input on port 5006
% - Python settings packet on port 5004
% - GeneralFilter.m for UWB position filtering
% - Optional ImuFusionFilter.m path using ReadLimo.m or ReadBebop.m
% - final_position and final_angles selection inside MATLAB
% - Optional ControlLimo.m or ControlBebop.m update interface
% - Optional ROS publishing of final_position and final_angles
% - Useful logging for testing/debugging
%
% Placeholders:
% - Custom PCB data ROS input is kept in ReadCustomPcb.m and has not been
% integrated into the Master script yet.
%
% Future Work:
% - Currently both GPS-RTK and OptiTrack send their source as 'OptiTrack' 
% in the packets. This is not very clean. In the future they should send
% their own names and this script should be able to receive both and use
% them as ground truth.
% - 'Send Data To ROS' and 'Control Robots' from the python GUI are
% currently not being sent to Matlab, these settings have to be enabled
% manually. Matlab variables: USE_ROS_PUBLISHING & USE_ROBOT_CONTROL.
%
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
% These settings are intentionally controlled inside MATLAB. Python starts
% the measurement/session, but MATLAB decides which filtered source is used
% as final_position.

% Final position options:
% "UWB_GENERAL" = UWB after GeneralFilter only
% "UWB_IMU"     = UWB after GeneralFilter + ImuFusionFilter
% "OPTITRACK"   = OptiTrack ground truth used as final_position
FINAL_POSITION_SOURCE = "UWB_GENERAL";

% UWB input source used by this master script.
UWB_SOURCE_MATLAB = "PYTHON_UDP"; % PYTHON_UDP or CUSTOM_PCB_ROS

% IMU / robot selection.
% SELECTED_ROBOT options: "NONE", "LIMO", "BEBOP"
USE_IMU_FILTER = true;
SELECTED_ROBOT = "NONE";

% Robot control. This only calls the updated ControlLimo/ControlBebop
% interface
USE_ROBOT_CONTROL = false;
START_ROBOT_MOVEMENT_AUTOMATICALLY = false;
CONTROL_UPDATE_RATE = 50;       % [Hz]

% Optional ROS publishing of final output. This is separate from robot
% control. It can be enabled for debugging or for other ROS nodes.
USE_ROS_PUBLISHING = false;
ROS_MASTER_ADDRESS = "";         % empty = rosinit() default
ROS_POSITION_TOPIC = "/uwb/final_position";
ROS_ANGLES_TOPIC = "/uwb/final_angles";
ROS_FRAME_ID = "map";
ROS_PUBLISH_RATE = 50;           % [Hz]

% Robot namespaces/topics, they must match the namespace used in ROS
LIMO_NAMESPACE = "/L1";
BEBOP_NAMESPACE = "/B1";         % reader namespace, including slash
BEBOP_CONTROL_NAMESPACE = "B1";  % control class namespace, no slash

% Runtime settings
T_exp = 2400;                    % maximum runtime [s]
T_final_log = 1/30;              % final_position logging rate [s]
MAIN_LOOP_PAUSE = 0.001;         % small pause to avoid maxing CPU

% Startup sensor wait
WAIT_FOR_REQUIRED_SENSORS = true;
SENSOR_WAIT_TIMEOUT = 30;        % [s]
SENSOR_FRESH_TIMEOUT = 2.0;      % [s]
IMU_FRESH_TIMEOUT = 0.5;         % [s]

% Filter settings
FILTER_DEFAULT_DT = 0.1;
ENABLE_GENERAL_FILTER_LOG = true;
ENABLE_IMU_FILTER_LOG = true;

% Status printing in command window, speed can be throttled
PRINT_EVERY_UWB_PACKETS = 10;
PRINT_EVERY_OPTI_PACKETS = 200;
PRINT_EVERY_IMU_PACKETS = 50;
PRINT_EVERY_FINAL_LINES = 30;



%% 2. DERIVED CONFIGURATION AND SAFETY CHECKS
% =========================================================================
FINAL_POSITION_SOURCE = upper(string(FINAL_POSITION_SOURCE));
UWB_SOURCE_MATLAB = upper(string(UWB_SOURCE_MATLAB));
SELECTED_ROBOT = upper(string(SELECTED_ROBOT));

if ~any(FINAL_POSITION_SOURCE == ["UWB_GENERAL", "UWB_IMU", "OPTITRACK"])
    fprintf('[MASTER] Invalid FINAL_POSITION_SOURCE. Falling back to UWB_GENERAL.\n');
    FINAL_POSITION_SOURCE = "UWB_GENERAL";
end

if ~any(UWB_SOURCE_MATLAB == ["PYTHON_UDP", "CUSTOM_PCB_ROS"])
    fprintf('[MASTER] Invalid UWB_SOURCE_MATLAB. Falling back to PYTHON_UDP.\n');
    UWB_SOURCE_MATLAB = "PYTHON_UDP";
end

if ~any(SELECTED_ROBOT == ["NONE", "LIMO", "BEBOP"])
    fprintf('[MASTER] Invalid SELECTED_ROBOT. Falling back to NONE.\n');
    SELECTED_ROBOT = "NONE";
end

if SELECTED_ROBOT == "NONE"
    USE_IMU_FILTER = false;
    if USE_ROBOT_CONTROL
        fprintf('[MASTER] SELECTED_ROBOT is NONE, so robot control is disabled.\n');
    end
    USE_ROBOT_CONTROL = false;
end

if FINAL_POSITION_SOURCE == "UWB_IMU" && SELECTED_ROBOT == "NONE"
    fprintf('[MASTER] Warning: FINAL_POSITION_SOURCE is UWB_IMU, but SELECTED_ROBOT is NONE.\n');
    fprintf('[MASTER] UWB_IMU will fall back to GeneralFilter output until an IMU source is selected.\n');
end

USE_IMU_FILTER_OBJECT = USE_IMU_FILTER || FINAL_POSITION_SOURCE == "UWB_IMU";
USE_ROBOT_READER = SELECTED_ROBOT ~= "NONE" && ...
    (USE_IMU_FILTER || USE_ROBOT_CONTROL || FINAL_POSITION_SOURCE == "UWB_IMU");
USE_ROS = USE_IMU_FILTER || USE_ROBOT_CONTROL || USE_ROS_PUBLISHING || USE_ROBOT_READER;

T_control = 1 / max(CONTROL_UPDATE_RATE, 1);
T_ros_publish = 1 / max(ROS_PUBLISH_RATE, 1);



%% 3. UDP PORT SETUP
% =========================================================================
SETTINGS_PORT = 5004;
UWB_PORT = 5005;
OPTI_PORT = 5006;

fprintf('[MASTER] Opening UDP ports...\n');
try
    u_settings = udpport("LocalHost", "127.0.0.1", "LocalPort", SETTINGS_PORT);
    u_uwb = udpport("LocalHost", "127.0.0.1", "LocalPort", UWB_PORT);
    u_opti = udpport("LocalHost", "127.0.0.1", "LocalPort", OPTI_PORT);
catch ME
    fprintf('[MASTER] Failed to open UDP ports: %s\n', ME.message);
    fprintf('[MASTER] Make sure no old MATLAB process is still using ports %d, %d or %d.\n', ...
        SETTINGS_PORT, UWB_PORT, OPTI_PORT);
    return;
end

fprintf('[MASTER] Settings UDP: 127.0.0.1:%d\n', SETTINGS_PORT);
fprintf('[MASTER] UWB UDP:      127.0.0.1:%d\n', UWB_PORT);
fprintf('[MASTER] Opti UDP:     127.0.0.1:%d\n\n', OPTI_PORT);



%% 4. WAIT FOR SETTINGS PACKET FROM PYTHON
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



%% 5. APPLY SETTINGS FROM PYTHON, BUT KEEP FINAL POSITION SOURCE IN MATLAB
% =========================================================================
uwb_settings = getFieldDefault(settings, 'uwb', struct());
gt_settings = getFieldDefault(settings, 'ground_truth', struct());

session_name = string(getFieldDefault(settings, 'session_name', "Session_Unknown"));
session_dir = string(getFieldDefault(settings, 'session_dir', pwd));

ENABLE_UWB = toLogical(getFieldDefault(uwb_settings, 'enabled', false));
PYTHON_UWB_SOURCE = string(getFieldDefault(uwb_settings, 'source', "Listener"));
UWB_READ_TYPE = string(getFieldDefault(uwb_settings, 'read_type', "Tag Position"));
UWB_NETWORK_SCALE = string(getFieldDefault(uwb_settings, 'network_scale', "1 Network / 1 Listener"));

ENABLE_GT = toLogical(getFieldDefault(gt_settings, 'enabled', false));
GT_SOURCE = string(getFieldDefault(gt_settings, 'type', "OptiTrack"));

if ~isfolder(session_dir)
    mkdir(session_dir);
end

% Current master script uses Python listener UDP for active UWB input.
% Custom PCB input stays as a placeholder in ReadCustomPcb.m.
if ENABLE_UWB && UWB_SOURCE_MATLAB == "PYTHON_UDP" && PYTHON_UWB_SOURCE ~= "Listener"
    fprintf('[MASTER] Warning: Python selected UWB source "%s".\n', PYTHON_UWB_SOURCE);
    fprintf('[MASTER] This master currently uses Python listener UDP for active UWB input. UWB disabled.\n');
    ENABLE_UWB = false;
end

if UWB_SOURCE_MATLAB == "CUSTOM_PCB_ROS"
    fprintf('[MASTER] Warning: CUSTOM_PCB_ROS is still a placeholder in this master script.\n');
    fprintf('[MASTER] Active UWB UDP input is disabled until ReadCustomPcb.m is implemented.\n');
    ENABLE_UWB = false;
end

% Current master script uses OptiTrack UDP as active ground truth input.
if ENABLE_GT && GT_SOURCE ~= "OptiTrack"
    fprintf('[MASTER] Warning: Python selected ground truth "%s".\n', GT_SOURCE);
    fprintf('[MASTER] This master currently uses OptiTrack UDP as active ground truth input. Ground truth disabled.\n');
    ENABLE_GT = false;
end

fprintf('\n[MASTER] Session name: %s\n', session_name);
fprintf('[MASTER] Session folder: %s\n', session_dir);
fprintf('[MASTER] MATLAB UWB source: %s\n', UWB_SOURCE_MATLAB);
fprintf('[MASTER] Python UWB enabled: %d | Read mode: %s | Network scale: %s\n', ...
    ENABLE_UWB, UWB_READ_TYPE, UWB_NETWORK_SCALE);
fprintf('[MASTER] OptiTrack enabled: %d\n', ENABLE_GT);
fprintf('[MASTER] Final position source: %s\n', FINAL_POSITION_SOURCE);
fprintf('[MASTER] Selected robot: %s | IMU filter: %d | Robot control: %d | ROS publishing: %d\n\n', ...
    SELECTED_ROBOT, USE_IMU_FILTER, USE_ROBOT_CONTROL, USE_ROS_PUBLISHING);



%% 6. ROS SETUP, READERS, FILTERS, CONTROLLERS AND LOGGING
% =========================================================================
ros_available = initializeRosIfNeeded(USE_ROS, ROS_MASTER_ADDRESS);

if USE_ROS && ~ros_available
    fprintf('[MASTER] ROS-dependent features are disabled because ROS could not be initialized.\n');
    USE_IMU_FILTER = false;
    USE_ROBOT_CONTROL = false;
    USE_ROS_PUBLISHING = false;
    USE_ROBOT_READER = false;

    if FINAL_POSITION_SOURCE == "UWB_IMU"
        fprintf('[MASTER] FINAL_POSITION_SOURCE changed from UWB_IMU to UWB_GENERAL.\n');
        FINAL_POSITION_SOURCE = "UWB_GENERAL";
    end
end

% -------------------------- GeneralFilter -------------------------------
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

% -------------------------- ImuFusionFilter -----------------------------
uwb_imu_filter = [];
if ENABLE_IMU_FILTER_LOG
    imu_filter_log = fullfile(session_dir, "[Log]_uwb_imu_filter_" + session_name + ".csv");
else
    imu_filter_log = '';
end

if USE_IMU_FILTER_OBJECT
    imu_config = makeImuFilterConfig(USE_IMU_FILTER, SELECTED_ROBOT, IMU_FRESH_TIMEOUT, ENABLE_IMU_FILTER_LOG);
    uwb_imu_filter = ImuFusionFilter(imu_config, imu_filter_log);

    if ENABLE_IMU_FILTER_LOG
        fprintf('[MASTER] ImuFusionFilter log: %s\n', imu_filter_log);
    end
end

% -------------------------- Robot IMU reader ----------------------------
imu_reader = [];
if USE_ROBOT_READER
    imu_reader = createRobotReader(SELECTED_ROBOT, LIMO_NAMESPACE, BEBOP_NAMESPACE);
end

% -------------------------- Robot controller ----------------------------
robot_controller = [];
if USE_ROBOT_CONTROL
    robot_controller = createRobotController(SELECTED_ROBOT, BEBOP_CONTROL_NAMESPACE);
end

% -------------------------- ROS publishers ------------------------------
ros_publishers = makeEmptyRosPublishers();
if USE_ROS_PUBLISHING
    ros_publishers = setupRosPublishers(ROS_POSITION_TOPIC, ROS_ANGLES_TOPIC);
    USE_ROS_PUBLISHING = ros_publishers.enabled;
end

% -------------------------- Final output log ----------------------------
final_log_path = fullfile(session_dir, "[Log]_final_position_matlab_" + session_name + ".csv");
final_log_fid = openFinalPositionLog(final_log_path);

if final_log_fid > 0
    fprintf('[MASTER] Final position log: %s\n\n', final_log_path);
else
    fprintf('[MASTER] Warning: final_position logging disabled.\n\n');
end



%% 7. STATE VARIABLES
% =========================================================================
t_exp = tic;
t_final_log = tic;
t_control_update = tic;
t_ros_publish = tic;

latest_uwb_sample = makeEmptyUwbSample();
latest_uwb_general = makeEmptyGeneralFilteredSample();
latest_uwb_imu = makeEmptyImuFilteredSample();
latest_opti_sample = makeEmptyOptiSample();
latest_imu_sample = makeEmptyImuSample();
final_position = makeEmptyFinalPosition();
final_angles = makeEmptyAngles();

last_uwb_receive_time = -Inf;
last_opti_receive_time = -Inf;
last_imu_receive_time = -Inf;
last_processed_imu_timestamp = NaN;

print_counter_uwb = 0;
print_counter_opti = 0;
print_counter_imu = 0;
print_counter_final = 0;

require_uwb = ENABLE_UWB && any(FINAL_POSITION_SOURCE == ["UWB_GENERAL", "UWB_IMU"]);
require_opti = ENABLE_GT && FINAL_POSITION_SOURCE == "OPTITRACK";
require_imu = USE_IMU_FILTER && USE_ROBOT_READER;



%% 8. WAIT FOR REQUIRED SENSORS
% =========================================================================
if WAIT_FOR_REQUIRED_SENSORS
    disp('[MASTER] Waiting for required sensors to become active...');
    sensor_wait_start = tic;

    while toc(sensor_wait_start) < SENSOR_WAIT_TIMEOUT
        [latest_uwb_sample, latest_uwb_general, last_uwb_receive_time, print_counter_uwb, new_uwb_general] = ...
            receiveAndFilterUwb(u_uwb, uwb_general_filter, ENABLE_UWB, t_exp, ...
            latest_uwb_sample, latest_uwb_general, last_uwb_receive_time, print_counter_uwb, PRINT_EVERY_UWB_PACKETS);

        [latest_opti_sample, last_opti_receive_time, print_counter_opti] = ...
            receiveOptiTrack(u_opti, ENABLE_GT, t_exp, latest_opti_sample, ...
            last_opti_receive_time, print_counter_opti, PRINT_EVERY_OPTI_PACKETS);

        [latest_imu_sample, final_angles, last_imu_receive_time, print_counter_imu, new_imu_sample] = ...
            readRobotImu(imu_reader, USE_ROBOT_READER, t_exp, latest_imu_sample, final_angles, ...
            last_imu_receive_time, print_counter_imu, PRINT_EVERY_IMU_PACKETS);

        if USE_IMU_FILTER_OBJECT
            [latest_uwb_imu, last_processed_imu_timestamp] = updateImuFilter(uwb_imu_filter, ...
                latest_uwb_general, latest_uwb_imu, latest_imu_sample, ...
                new_uwb_general, new_imu_sample, last_processed_imu_timestamp);
        end

        current_time = toc(t_exp);
        uwb_ready = ~require_uwb || isSensorFresh(last_uwb_receive_time, current_time, SENSOR_FRESH_TIMEOUT);
        opti_ready = ~require_opti || isSensorFresh(last_opti_receive_time, current_time, SENSOR_FRESH_TIMEOUT);
        imu_ready = ~require_imu || isSensorFresh(last_imu_receive_time, current_time, SENSOR_FRESH_TIMEOUT);

        if uwb_ready && opti_ready && imu_ready
            disp('[MASTER] Required sensors are active.');
            break;
        end

        if require_uwb && ~uwb_ready
            fprintf('[MASTER] Waiting for UWB listener data on port %d...\n', UWB_PORT);
        end
        if require_opti && ~opti_ready
            fprintf('[MASTER] Waiting for OptiTrack data on port %d...\n', OPTI_PORT);
        end
        if require_imu && ~imu_ready
            fprintf('[MASTER] Waiting for %s IMU data...\n', SELECTED_ROBOT);
        end

        pause(0.5);
    end

    current_time = toc(t_exp);
    uwb_ready = ~require_uwb || isSensorFresh(last_uwb_receive_time, current_time, SENSOR_FRESH_TIMEOUT);
    opti_ready = ~require_opti || isSensorFresh(last_opti_receive_time, current_time, SENSOR_FRESH_TIMEOUT);
    imu_ready = ~require_imu || isSensorFresh(last_imu_receive_time, current_time, SENSOR_FRESH_TIMEOUT);

    if ~(uwb_ready && opti_ready && imu_ready)
        disp('[MASTER] Warning: not all required sensors became active before timeout.');
        disp('[MASTER] Script will continue for logging/debugging.');
    else
        disp('[MASTER] Sensor startup check complete.');
    end
end



%% 9. START ROBOT MOVEMENT IF ENABLED
% =========================================================================
if USE_ROBOT_CONTROL && START_ROBOT_MOVEMENT_AUTOMATICALLY && ~isempty(robot_controller)
    try
        robot_controller.perform_movement();
    catch ME
        fprintf('[MASTER] Could not start robot movement: %s\n', ME.message);
    end
end



%% 10. MAIN LOOP
% =========================================================================
disp('[MASTER] Entering main loop. Press Ctrl+C to stop.');

try
    while toc(t_exp) < T_exp
        % -------------------------------------------------------------
        % A. UWB from Python UDP -> GeneralFilter
        % -------------------------------------------------------------
        [latest_uwb_sample, latest_uwb_general, last_uwb_receive_time, print_counter_uwb, new_uwb_general] = ...
            receiveAndFilterUwb(u_uwb, uwb_general_filter, ENABLE_UWB, t_exp, ...
            latest_uwb_sample, latest_uwb_general, last_uwb_receive_time, print_counter_uwb, PRINT_EVERY_UWB_PACKETS);

        % -------------------------------------------------------------
        % B. OptiTrack from Python UDP
        % -------------------------------------------------------------
        [latest_opti_sample, last_opti_receive_time, print_counter_opti] = ...
            receiveOptiTrack(u_opti, ENABLE_GT, t_exp, latest_opti_sample, ...
            last_opti_receive_time, print_counter_opti, PRINT_EVERY_OPTI_PACKETS);

        % -------------------------------------------------------------
        % C. Robot IMU / angles from selected reader
        % -------------------------------------------------------------
        [latest_imu_sample, final_angles, last_imu_receive_time, print_counter_imu, new_imu_sample] = ...
            readRobotImu(imu_reader, USE_ROBOT_READER, t_exp, latest_imu_sample, final_angles, ...
            last_imu_receive_time, print_counter_imu, PRINT_EVERY_IMU_PACKETS);

        % -------------------------------------------------------------
        % D. Optional UWB + IMU filter path
        % -------------------------------------------------------------
        if USE_IMU_FILTER_OBJECT
            [latest_uwb_imu, last_processed_imu_timestamp] = updateImuFilter(uwb_imu_filter, ...
                latest_uwb_general, latest_uwb_imu, latest_imu_sample, ...
                new_uwb_general, new_imu_sample, last_processed_imu_timestamp);
        end

        % -------------------------------------------------------------
        % E. Select final_position inside MATLAB
        % -------------------------------------------------------------
        final_position = selectFinalPosition(FINAL_POSITION_SOURCE, latest_uwb_general, latest_uwb_imu, latest_opti_sample);

        % -------------------------------------------------------------
        % F. Robot control update
        % -------------------------------------------------------------
        if USE_ROBOT_CONTROL && ~isempty(robot_controller) && toc(t_control_update) >= T_control
            t_control_update = tic;
            updateRobotController(robot_controller, final_position, final_angles);
        end

        % -------------------------------------------------------------
        % G. Optional ROS publish
        % -------------------------------------------------------------
        if USE_ROS_PUBLISHING && toc(t_ros_publish) >= T_ros_publish
            t_ros_publish = tic;
            publishFinalRos(ros_publishers, final_position, final_angles, ROS_FRAME_ID);
        end

        % -------------------------------------------------------------
        % H. Log final_position at fixed rate
        % -------------------------------------------------------------
        if toc(t_final_log) >= T_final_log
            t_final_log = tic;
            logFinalPosition(final_log_fid, final_position, final_angles, ...
                latest_uwb_general, latest_uwb_imu, latest_opti_sample, ...
                USE_ROBOT_CONTROL, USE_ROS_PUBLISHING);

            print_counter_final = print_counter_final + 1;
            if mod(print_counter_final, PRINT_EVERY_FINAL_LINES) == 0 && final_position.valid
                fprintf('[FINAL] Source: %s -> X: %.3f, Y: %.3f, Z: %.3f | yaw: %.3f deg\n', ...
                    final_position.source, final_position.position(1), final_position.position(2), ...
                    final_position.position(3), final_angles.yaw);
            end
        end

        drawnow;
        pause(MAIN_LOOP_PAUSE);
    end

catch ME
    fprintf('[MASTER] CRASH: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('[MASTER] File: %s | Line: %d\n', ME.stack(1).file, ME.stack(1).line);
    end
end



%% 11. SAFE SHUTDOWN
% =========================================================================
disp('[MASTER] Cleaning up resources...');

try
    if USE_ROBOT_CONTROL && ~isempty(robot_controller)
        safeStopRobot(robot_controller, SELECTED_ROBOT);
    end
catch
end

try
    uwb_general_filter.closeLog();
catch
end

try
    if ~isempty(uwb_imu_filter)
        uwb_imu_filter.closeLog();
    end
catch
end

closeFileSafe(final_log_fid);

try
    if ~isempty(imu_reader)
        imu_reader.disconnect();
    end
catch
end

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
    if isfield(decoded, 'packet_type') && string(decoded.packet_type) == "settings"
        settings = decoded;
        valid = true;
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
function ros_available = initializeRosIfNeeded(use_ros, ros_master_address)
ros_available = false;

if ~use_ros
    return;
end

try
    if strlength(string(ros_master_address)) > 0
        rosinit(char(ros_master_address));
    else
        rosinit;
    end
    ros_available = true;
    fprintf('[MASTER] ROS initialized.\n');
catch ME
    fprintf('[MASTER] ROS initialization failed: %s\n', ME.message);
    ros_available = false;
end
end

% -------------------------------------------------------------------------
function imu_config = makeImuFilterConfig(use_imu_filter, selected_robot, imu_fresh_timeout, enable_log)
imu_config = struct();
imu_config.USE_IMU_FILTER = logical(use_imu_filter);
imu_config.FALLBACK_TO_GENERAL_FILTER = true;
imu_config.IMU_FRESH_TIMEOUT = imu_fresh_timeout;
imu_config.LOG_TO_CSV = logical(enable_log);

% Angles from ReadLimo.m and ReadBebop.m should be degrees.
% The ImuFusionFilter.m uses cosd/sind internally.
switch upper(string(selected_robot))
    case "LIMO"
        imu_config.ROBOT_MODE = 'GROUND_2D';
    case "BEBOP"
        imu_config.ROBOT_MODE = 'DRONE_3D';
    otherwise
        imu_config.ROBOT_MODE = 'GROUND_2D';
end
end

% -------------------------------------------------------------------------
function reader = createRobotReader(selected_robot, limo_namespace, bebop_namespace)
reader = [];
try
    switch upper(string(selected_robot))
        case "LIMO"
            cfg = struct();
            cfg.ROS_NAMESPACE = string(limo_namespace);
            cfg.AUTO_CONNECT = false;
            reader = ReadLimo(cfg);
            reader.connect();

        case "BEBOP"
            cfg = struct();
            cfg.ROS_NAMESPACE = string(bebop_namespace);
            cfg.AUTO_CONNECT = false;
            reader = ReadBebop(cfg);
            reader.connect();

        otherwise
            reader = [];
    end
catch ME
    fprintf('[MASTER] Could not create robot reader: %s\n', ME.message);
    reader = [];
end
end

% -------------------------------------------------------------------------
function controller = createRobotController(selected_robot, bebop_control_namespace)
controller = [];
try
    switch upper(string(selected_robot))
        case "LIMO"
            controller = ControlLimo();
        case "BEBOP"
            controller = ControlBebop(char(bebop_control_namespace));
        otherwise
            controller = [];
    end
catch ME
    fprintf('[MASTER] Could not create robot controller: %s\n', ME.message);
    controller = [];
end
end

% -------------------------------------------------------------------------
function ros_publishers = makeEmptyRosPublishers()
ros_publishers.enabled = false;
ros_publishers.position_pub = [];
ros_publishers.position_msg = [];
ros_publishers.angles_pub = [];
ros_publishers.angles_msg = [];
end

% -------------------------------------------------------------------------
function ros_publishers = setupRosPublishers(position_topic, angles_topic)
ros_publishers = makeEmptyRosPublishers();

try
    ros_publishers.position_pub = rospublisher(char(position_topic), 'geometry_msgs/PoseStamped');
    ros_publishers.position_msg = rosmessage(ros_publishers.position_pub);

    ros_publishers.angles_pub = rospublisher(char(angles_topic), 'geometry_msgs/Vector3Stamped');
    ros_publishers.angles_msg = rosmessage(ros_publishers.angles_pub);

    ros_publishers.enabled = true;
    fprintf('[MASTER] ROS publishing enabled: %s and %s\n', char(position_topic), char(angles_topic));

catch ME
    fprintf('[MASTER] ROS publisher setup failed: %s\n', ME.message);
    ros_publishers = makeEmptyRosPublishers();
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
function [latest_uwb_sample, latest_uwb_general, last_receive_time, print_counter, new_sample] = ...
    receiveAndFilterUwb(u_uwb, general_filter, enable_uwb, t_exp, ...
    latest_uwb_sample, latest_uwb_general, last_receive_time, print_counter, print_every)

new_sample = false;

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
new_sample = true;
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
function [latest_imu_sample, final_angles, last_receive_time, print_counter, new_sample] = ...
    readRobotImu(reader, use_robot_reader, t_exp, latest_imu_sample, final_angles, ...
    last_receive_time, print_counter, print_every)

new_sample = false;

if ~use_robot_reader || isempty(reader)
    return;
end

try
    [imu_sample, angles] = reader.getLatest();
catch ME
    fprintf('[MASTER] Robot IMU read error: %s\n', ME.message);
    return;
end

if ~isstruct(imu_sample) || ~isfield(imu_sample, 'valid') || ~imu_sample.valid
    return;
end

if isfield(latest_imu_sample, 'timestamp') && isfinite(latest_imu_sample.timestamp) && ...
   isfield(imu_sample, 'timestamp') && isfinite(imu_sample.timestamp) && ...
   imu_sample.timestamp == latest_imu_sample.timestamp
    return;
end

latest_imu_sample = imu_sample;
final_angles = angles;
last_receive_time = toc(t_exp);
new_sample = true;

print_counter = print_counter + 1;
if mod(print_counter, print_every) == 0
    fprintf('[IMU] Source: %s | accel [%.2f %.2f %.2f] | rpy [%.3f %.3f %.3f] deg\n', ...
        char(string(imu_sample.source)), ...
        imu_sample.accel_body(1), imu_sample.accel_body(2), imu_sample.accel_body(3), ...
        final_angles.roll, final_angles.pitch, final_angles.yaw);
end
end

% -------------------------------------------------------------------------
function [latest_uwb_imu, last_processed_imu_timestamp] = updateImuFilter(uwb_imu_filter, ...
    latest_uwb_general, latest_uwb_imu, latest_imu_sample, new_uwb_general, new_imu_sample, last_processed_imu_timestamp)

if isempty(uwb_imu_filter)
    return;
end

try
    if new_imu_sample && latest_imu_sample.valid
        current_imu_timestamp = latest_imu_sample.timestamp;
        if ~isfinite(last_processed_imu_timestamp) || current_imu_timestamp ~= last_processed_imu_timestamp
            uwb_imu_filter.processImuSample(latest_imu_sample);
            latest_uwb_imu = uwb_imu_filter.getOutput();
            last_processed_imu_timestamp = current_imu_timestamp;
        end
    end

    if new_uwb_general && latest_uwb_general.valid
        latest_uwb_imu = uwb_imu_filter.processUwbSample(latest_uwb_general);
    end

catch ME
    fprintf('[MASTER] ImuFusionFilter update error: %s\n', ME.message);
end
end

% -------------------------------------------------------------------------
function final_position = selectFinalPosition(final_source, uwb_general, uwb_imu, opti_sample)
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

    case "UWB_IMU"
        if uwb_imu.valid
            final_position.position = uwb_imu.position;
            final_position.velocity = uwb_imu.velocity;
            final_position.timestamp = uwb_imu.timestamp;
            final_position.valid = true;
            final_position.source = "uwb_imu";
            final_position.quality = uwb_imu.quality;
        elseif uwb_general.valid
            final_position.position = uwb_general.position;
            final_position.velocity = uwb_general.velocity;
            final_position.timestamp = uwb_general.timestamp;
            final_position.valid = true;
            final_position.source = "uwb_imu_fallback_general";
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
function updateRobotController(controller, final_position, final_angles)
if isempty(controller) || ~final_position.valid
    return;
end

try
    controller.update(final_position, final_angles);
catch ME
    fprintf('[MASTER] Robot control update error: %s\n', ME.message);
end
end

% -------------------------------------------------------------------------
function publishFinalRos(ros_publishers, final_position, final_angles, frame_id)
if ~ros_publishers.enabled || ~final_position.valid
    return;
end

try
    % Position + orientation
    pos_msg = ros_publishers.position_msg;
    pos_msg.Header.Stamp = rostime('now');
    pos_msg.Header.FrameId = char(frame_id);

    pos_msg.Pose.Position.X = final_position.position(1);
    pos_msg.Pose.Position.Y = final_position.position(2);
    pos_msg.Pose.Position.Z = final_position.position(3);

    if isfield(final_angles, 'valid') && final_angles.valid
        q = rpyToQuaternion(final_angles.roll, final_angles.pitch, final_angles.yaw);
    else
        q = [1 0 0 0];
    end

    pos_msg.Pose.Orientation.W = q(1);
    pos_msg.Pose.Orientation.X = q(2);
    pos_msg.Pose.Orientation.Y = q(3);
    pos_msg.Pose.Orientation.Z = q(4);
    send(ros_publishers.position_pub, pos_msg);

    % Angles as roll/pitch/yaw vector in degrees.
    ang_msg = ros_publishers.angles_msg;
    ang_msg.Header.Stamp = rostime('now');
    ang_msg.Header.FrameId = char(frame_id);
    ang_msg.Vector.X = final_angles.roll;
    ang_msg.Vector.Y = final_angles.pitch;
    ang_msg.Vector.Z = final_angles.yaw;
    send(ros_publishers.angles_pub, ang_msg);

catch ME
    fprintf('[MASTER] ROS publish error: %s\n', ME.message);
end
end

% -------------------------------------------------------------------------
function q = rpyToQuaternion(roll_deg, pitch_deg, yaw_deg)
% Input angles are degrees. Returns quaternion as [w x y z].
if ~all(isfinite([roll_deg, pitch_deg, yaw_deg]))
    q = [1 0 0 0];
    return;
end

roll = deg2rad(roll_deg);
pitch = deg2rad(pitch_deg);
yaw = deg2rad(yaw_deg);

cy = cos(yaw * 0.5);
sy = sin(yaw * 0.5);
cp = cos(pitch * 0.5);
sp = sin(pitch * 0.5);
cr = cos(roll * 0.5);
sr = sin(roll * 0.5);

qw = cr * cp * cy + sr * sp * sy;
qx = sr * cp * cy - cr * sp * sy;
qy = cr * sp * cy + sr * cp * sy;
qz = cr * cp * sy - sr * sp * cy;

q = [qw qx qy qz];
end

% -------------------------------------------------------------------------
function safeStopRobot(controller, selected_robot)
try
    switch upper(string(selected_robot))
        case "LIMO"
            if ismethod(controller, 'stop')
                controller.stop();
            elseif ismethod(controller, 'landAndStop')
                controller.landAndStop();
            end

        case "BEBOP"
            if ismethod(controller, 'landAndStop')
                controller.landAndStop();
            elseif ismethod(controller, 'hover')
                controller.hover();
            end
    end
catch ME
    fprintf('[MASTER] Robot safe stop error: %s\n', ME.message);
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
            'final_valid,final_quality,roll_deg,pitch_deg,yaw_deg,angles_valid,angles_source,', ...
            'uwb_general_x,uwb_general_y,uwb_general_z,uwb_accepted,uwb_rejection_reason,', ...
            'uwb_imu_x,uwb_imu_y,uwb_imu_z,uwb_imu_valid,uwb_imu_used_imu,uwb_imu_fallback,uwb_imu_status,', ...
            'opti_x,opti_y,opti_z,opti_valid,robot_control_enabled,ros_publish_enabled\n']);
    end

catch ME
    fprintf('[MASTER] Could not open final position log: %s\n', ME.message);
    fid = -1;
end
end

% -------------------------------------------------------------------------
function logFinalPosition(fid, final_position, final_angles, uwb_general, uwb_imu, opti_sample, robot_control_enabled, ros_publish_enabled)
if fid <= 0
    return;
end

fprintf(fid, ['%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,', ...
    '%.6f,%.6f,%.6f,%d,%s,', ...
    '%.6f,%.6f,%.6f,%d,%s,', ...
    '%.6f,%.6f,%.6f,%d,%d,%d,%s,', ...
    '%.6f,%.6f,%.6f,%d,%d,%d\n'], ...
    final_position.timestamp, ...
    safeCsvText(final_position.source), ...
    final_position.position(1), final_position.position(2), final_position.position(3), ...
    final_position.velocity(1), final_position.velocity(2), final_position.velocity(3), ...
    final_position.valid, ...
    final_position.quality, ...
    final_angles.roll, final_angles.pitch, final_angles.yaw, ...
    final_angles.valid, ...
    safeCsvText(final_angles.source), ...
    uwb_general.position(1), uwb_general.position(2), uwb_general.position(3), ...
    getStructFieldDefault(uwb_general, 'accepted', false), ...
    safeCsvText(getStructFieldDefault(uwb_general, 'rejection_reason', 'none')), ...
    uwb_imu.position(1), uwb_imu.position(2), uwb_imu.position(3), ...
    uwb_imu.valid, ...
    getStructFieldDefault(uwb_imu, 'used_imu', false), ...
    getStructFieldDefault(uwb_imu, 'fallback_used', true), ...
    safeCsvText(getStructFieldDefault(uwb_imu, 'status', 'none')), ...
    opti_sample.position(1), opti_sample.position(2), opti_sample.position(3), ...
    opti_sample.valid, ...
    robot_control_enabled, ...
    ros_publish_enabled);
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
function out = makeEmptyImuFilteredSample()
out.position = [NaN, NaN, NaN];
out.velocity = [NaN, NaN, NaN];
out.orientation = [NaN, NaN, NaN];
out.timestamp = NaN;
out.quality = NaN;
out.valid = false;
out.source = "imu_fusion_filter";
out.used_imu = false;
out.fallback_used = true;
out.status = "not_initialized";
out.input_uwb_source = "none";
out.input_uwb_valid = false;
out.input_uwb_accepted = false;
out.input_uwb_rejection_reason = "none";
out.imu_valid = false;
out.imu_source = "none";
out.imu_timestamp = NaN;
out.accel_body = [NaN, NaN, NaN];
out.gyro_body = [NaN, NaN, NaN];
out.debug = struct();
end

% -------------------------------------------------------------------------
function imu = makeEmptyImuSample()
imu.accel_body = [NaN, NaN, NaN];
imu.gyro_body = [NaN, NaN, NaN];
imu.orientation = [NaN, NaN, NaN];
imu.timestamp = NaN;
imu.valid = false;
imu.source = "none";
end

% -------------------------------------------------------------------------
function angles = makeEmptyAngles()
angles.roll = NaN;
angles.pitch = NaN;
angles.yaw = NaN;
angles.timestamp = NaN;
angles.valid = false;
angles.source = "none";
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