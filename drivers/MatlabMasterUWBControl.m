% =========================================================================
% MATLAB MASTER UWB CONTROL
% Author: Renzo Eisma
% Date: 09/04/2026
%
% Description:
% Master control script for receiving settings, UWB data and ground-truth
% data from the Python software framework. The script filters the incoming
% UWB position, logs the MATLAB-filtered UWB data, optionally publishes
% position data to ROS and optionally sends robot control commands.
%
% New framework:
%   Port 5004 = Settings/config packet from MasterControlStation.py
%   Port 5005 = Live UWB packets from uwb_sensor.py
%   Port 5006 = Live OptiTrack packets from NatNetClient.py
% =========================================================================

function MatlabMasterUWBControl()

%% 0. SYSTEM CLEANUP & INIT
% =====================================================================================================================
clear classes;
close all;
clc;

try
    rosshutdown;
catch
end
pause(0.5); % Ensure old ROS connections are killed immediately to clear ghost nodes


%% 1. MANUAL SYSTEM CONFIGURATION
% =====================================================================================================================
% Robot selection is intentionally kept manual for now.
SelectedRobot = "Bebop2";      % "Bebop2", "Crazyflie", "Limu"

% ROS master IP is also kept manual for now.
ros_ip = '192.168.0.100';

% General timing settings
T_exp = 2400;                  % Maximum runtime [s]
T_run = 1/30;                  % Robot/control update rate [s]
movement_time = 10;            % Safety movement time before stop [s]
SENSOR_WAIT_TIMEOUT = 30;      % Maximum time to wait for required sensors [s]
SENSOR_FRESH_TIMEOUT = 2.0;    % Sensor is considered alive if last packet is newer than this [s]

% Accelerometer reading is only used through robot classes in MATLAB.
ENABLE_ACCEL_READ = true;


%% 2. UDP PORT SETUP
% =====================================================================================================================
disp('[MASTER] Opening UDP Ports...');

try
    u_settings = udpport("LocalHost", "127.0.0.1", "LocalPort", 5004);
    u_uwb      = udpport("LocalHost", "127.0.0.1", "LocalPort", 5005);
    u_opti     = udpport("LocalHost", "127.0.0.1", "LocalPort", 5006);
catch ME
    fprintf('[MASTER] Failed to open UDP ports: %s\n', ME.message);
    disp('[MASTER] Make sure no old MATLAB process is still using ports 5004, 5005 or 5006.');
    return;
end

fprintf('[MASTER] Settings UDP: 127.0.0.1:%d\n', 5004);
fprintf('[MASTER] UWB UDP:      127.0.0.1:%d\n', 5005);
fprintf('[MASTER] Opti UDP:     127.0.0.1:%d\n', 5006);


%% 3. WAIT FOR SETTINGS PACKET FROM MASTERCONTROLSTATION
% =====================================================================================================================
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


%% 4. APPLY SETTINGS FROM PYTHON
% =====================================================================================================================
% The Python GUI decides what is enabled. MATLAB uses these settings to decide
% which sensors to wait for and whether ROS / robot control should start.

uwb_settings = getFieldDefault(settings, 'uwb', struct());
gt_settings = getFieldDefault(settings, 'ground_truth', struct());
routing_settings = getFieldDefault(settings, 'routing', struct());

session_name = string(getFieldDefault(settings, 'session_name', "Session_Unknown"));
session_dir = string(getFieldDefault(settings, 'session_dir', pwd));

ENABLE_UWB = toLogical(getFieldDefault(uwb_settings, 'enabled', false));
UWB_SOURCE = string(getFieldDefault(uwb_settings, 'source', "Listener"));
UWB_READ_TYPE = string(getFieldDefault(uwb_settings, 'read_type', "Tag Position"));
UWB_NETWORK_SCALE = string(getFieldDefault(uwb_settings, 'network_scale', "1 Network / 1 Listener"));

ENABLE_GT = toLogical(getFieldDefault(gt_settings, 'enabled', false));
GT_SOURCE = string(getFieldDefault(gt_settings, 'type', "OptiTrack"));
GPS_RTK_TOPIC = string(getFieldDefault(gt_settings, 'gps_rtk_topic', "/gps/rtk"));
GPS_RTK_FRAME = string(getFieldDefault(gt_settings, 'gps_rtk_frame', "map"));

ENABLE_ROS = toLogical(getFieldDefault(routing_settings, 'send_data_to_ros', false));
ENABLE_ROBOT_CONTROL = toLogical(getFieldDefault(routing_settings, 'control_robots', false));

% Future UWB-over-ROS input settings.
% If UWB is read from ROS, MATLAB must initialize ROS even if it is not publishing to ROS.
UWB_ROS_TOPIC = string(getFieldDefault(uwb_settings, 'ros_topic', "/uwb/raw_position"));
UWB_ROS_MSG_TYPE = string(getFieldDefault(uwb_settings, 'ros_msg_type', "geometry_msgs/PointStamped"));

if ENABLE_UWB && UWB_SOURCE == "ROS" && ~ENABLE_ROS
    disp('[MASTER] UWB source is ROS, so ROS initialization is forced on.');
    ENABLE_ROS = true;
end

if ENABLE_ROBOT_CONTROL && ~ENABLE_ROS
    disp('[MASTER] Warning: Control Robots was enabled, but Send Data to ROS was disabled.');
    disp('[MASTER] Robot control requires ROS, so robot control is disabled.');
    ENABLE_ROBOT_CONTROL = false;
end

if ~isfolder(session_dir)
    mkdir(session_dir);
end

fprintf('\n[MASTER] Session name: %s\n', session_name);
fprintf('[MASTER] Session folder: %s\n', session_dir);
fprintf('[MASTER] UWB enabled: %d | Source: %s | Read mode: %s | Network scale: %s\n', ...
    ENABLE_UWB, UWB_SOURCE, UWB_READ_TYPE, UWB_NETWORK_SCALE);
fprintf('[MASTER] Ground truth enabled: %d | Source: %s\n', ENABLE_GT, GT_SOURCE);
fprintf('[MASTER] ROS enabled: %d | Robot control enabled: %d\n\n', ENABLE_ROS, ENABLE_ROBOT_CONTROL);


%% 5. MATLAB FILTER AND FILE LOGGING SETUP
% =====================================================================================================================
uwb_filter = FilterUWB(0.1);
fid = -1;
fileCleaner = [];

if ENABLE_UWB
    csv_name = "[Log]_uwbFilteredMatlab_listener1_" + session_name + ".csv";
    log_filename = fullfile(session_dir, csv_name);

    fid = fopen(log_filename, 'w');

    if fid ~= -1
        fprintf(fid, 'Time,Filt_X,Filt_Y,Filt_Z\n');
        disp(['[MASTER] MATLAB filtered UWB log initialized: ', char(log_filename)]);
        fileCleaner = onCleanup(@() closeFileSafe(fid));
    else
        disp('[MASTER] Warning: Could not open MATLAB filtered UWB CSV. Logging disabled.');
    end
else
    disp('[MASTER] UWB disabled. MATLAB filtered UWB CSV will not be created.');
end


%% 6. ROS / ROBOT SETUP
% =====================================================================================================================
robot = [];
ROS_INITIALIZED = false;
robot_is_moving = false;
current_accel = [0, 0, 0];
ros_publishers = struct();

% Placeholder object for future UWB input through ROS.
uwb_ros = struct();
uwb_ros.enabled = false;
uwb_ros.subscriber = [];
uwb_ros.topic = "";
uwb_ros.msg_type = "";

if ENABLE_ROS
    disp('[MASTER] Initializing ROS Network...');

    try
        try
            fclose(instrfindall);
        catch
        end

        rosshutdown;
        pause(1);
        rosinit(ros_ip);
        ROS_INITIALIZED = true;

        fprintf('[MASTER] ROS initialized. Selected Robot: %s\n', SelectedRobot);

        % Position publishers are intentionally simple. They are useful for debugging and later ROS integration.
        try
            ros_publishers.uwb = rospublisher('/uwb/filtered_position', 'geometry_msgs/Point');
            ros_publishers.uwb_msg = rosmessage(ros_publishers.uwb);

            ros_publishers.gt = rospublisher('/ground_truth/position', 'geometry_msgs/Point');
            ros_publishers.gt_msg = rosmessage(ros_publishers.gt);

            disp('[MASTER] ROS position publishers created.');
        catch ME
            fprintf('[MASTER] Warning: Could not create ROS publishers: %s\n', ME.message);
        end

        % Future UWB-over-ROS input.
        % This only creates the subscriber when UWB source is set to ROS.
        if ENABLE_UWB && UWB_SOURCE == "ROS"
            uwb_ros = setupUwbRosInput(UWB_ROS_TOPIC, UWB_ROS_MSG_TYPE);
        end

        % Robot class is only initialized when robot control is enabled.
        if ENABLE_ROBOT_CONTROL
            if SelectedRobot == "Limu"
                robot = RobotControlCodeLimuClass();
            elseif SelectedRobot == "Bebop2"
                robot = RobotControlCodeBebopClass();
            elseif SelectedRobot == "Crazyflie"
                robot = RobotControlCodeLimuClass(); % Placeholder until Crazyflie class exists
            end
        end

    catch ME
        fprintf('[MASTER] ROS Error: %s\n', ME.message);
        ENABLE_ROS = false;
        ENABLE_ROBOT_CONTROL = false;
        ENABLE_ACCEL_READ = false;
        ROS_INITIALIZED = false;
    end
end


%% 7. MATLAB/ROS SENSOR PLACEHOLDERS
% =====================================================================================================================
% Python currently live-logs UWB listener data and OptiTrack data.
% GPS RTK and custom PCB UWB-over-ROS are intended to be handled in MATLAB/ROS later.

if ENABLE_UWB && UWB_SOURCE == "ROS"
    fprintf('[MASTER] UWB-over-ROS selected. Topic: %s | Type: %s\n', UWB_ROS_TOPIC, UWB_ROS_MSG_TYPE);
end

if ENABLE_GT && GT_SOURCE == "GPS RTK"
    startGpsRtkPlaceholder(GPS_RTK_TOPIC, GPS_RTK_FRAME);
end


%% 8. MAIN LOOP VARIABLES
% =====================================================================================================================
latest_uwb_raw = [NaN, NaN, NaN];
latest_uwb_filt = [NaN, NaN, NaN];
latest_uwb_time = NaN;
latest_uwb_quality = NaN;
latest_uwb_position_type = "none";
last_uwb_receive_time = -Inf;

latest_opti = [NaN, NaN, NaN];
latest_opti_time = NaN;
latest_opti_quality = NaN;
last_opti_receive_time = -Inf;

print_counter_uwb = 0;
print_counter_opti = 0;
print_counter_accel = 0;

require_uwb_udp = ENABLE_UWB && UWB_SOURCE == "Listener";
require_uwb_ros = ENABLE_UWB && UWB_SOURCE == "ROS";
require_opti_udp = ENABLE_GT && GT_SOURCE == "OptiTrack";

if ENABLE_UWB && UWB_SOURCE == "ROS"
    disp('[MASTER] UWB source is ROS. UWB UDP waiting is skipped.');
elseif ENABLE_UWB && UWB_SOURCE ~= "Listener"
    disp('[MASTER] UWB source is not Listener. UWB UDP waiting is skipped for now.');
end

if ENABLE_GT && GT_SOURCE ~= "OptiTrack"
    disp('[MASTER] Ground-truth source is not OptiTrack. OptiTrack UDP waiting is skipped for now.');
end


%% 9. WAIT FOR REQUIRED SENSORS TO BECOME ACTIVE
% =====================================================================================================================
disp('[MASTER] Waiting for required sensors to become active...');

sensor_wait_start = tic;
t_exp = tic;

while toc(sensor_wait_start) < SENSOR_WAIT_TIMEOUT
    % Receive data while waiting, because Python or ROS may already be streaming.
    if ENABLE_UWB && UWB_SOURCE == "Listener"
        [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
            receiveAndProcessUwb(u_uwb, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb);

    elseif ENABLE_UWB && UWB_SOURCE == "ROS"
        [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
            receiveAndProcessUwbRos(uwb_ros, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb);
    end

    [latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti] = ...
        receiveAndProcessOpti(u_opti, t_exp, latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti);

    uwb_required = require_uwb_udp || require_uwb_ros;
    uwb_ready = ~uwb_required || isSensorFresh(last_uwb_receive_time, toc(t_exp), SENSOR_FRESH_TIMEOUT);
    opti_ready = ~require_opti_udp || isSensorFresh(last_opti_receive_time, toc(t_exp), SENSOR_FRESH_TIMEOUT);

    if uwb_ready && opti_ready
        disp('[MASTER] Required sensors are active.');
        break;
    end

    if require_uwb_udp && ~uwb_ready
        fprintf('[MASTER] Waiting for UWB listener data on port 5005...\n');
    elseif require_uwb_ros && ~uwb_ready
        fprintf('[MASTER] Waiting for UWB ROS data on topic %s...\n', UWB_ROS_TOPIC);
    end

    if require_opti_udp && ~opti_ready
        fprintf('[MASTER] Waiting for OptiTrack data on port 5006...\n');
    end

    pause(0.5);
end

uwb_required = require_uwb_udp || require_uwb_ros;
uwb_ready = ~uwb_required || isSensorFresh(last_uwb_receive_time, toc(t_exp), SENSOR_FRESH_TIMEOUT);
opti_ready = ~require_opti_udp || isSensorFresh(last_opti_receive_time, toc(t_exp), SENSOR_FRESH_TIMEOUT);

if ~(uwb_ready && opti_ready)
    disp('[MASTER] Warning: Not all required sensors became active before timeout.');
    disp('[MASTER] Logging/filtering will continue, but robot control is disabled for safety.');
    ENABLE_ROBOT_CONTROL = false;
else
    disp('[MASTER] Sensor startup check complete.');
end

% Start the robot movement only after sensors are confirmed and ROS is ready.
if ENABLE_ROBOT_CONTROL && ROS_INITIALIZED && ~isempty(robot)
    try
        robot.perform_movement();
        robot_is_moving = true;
        disp('[MASTER] Robot movement started.');
    catch ME
        fprintf('[MASTER] Could not start robot movement: %s\n', ME.message);
        ENABLE_ROBOT_CONTROL = false;
        robot_is_moving = false;
    end
elseif ENABLE_ROS && ROS_INITIALIZED
    disp('[MASTER] ROS is enabled, but robot movement is disabled.');
end


%% 10. HIGH SPEED EXECUTION LOOP
% =====================================================================================================================
disp('[MASTER] Entering main MATLAB control loop.');

t_run = tic;

try
    while toc(t_exp) < T_exp
        % ==========================================
        % A. UWB Reception + Filtering + Logging
        % ==========================================
        if ENABLE_UWB && UWB_SOURCE == "Listener"
            [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
                receiveAndProcessUwb(u_uwb, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb);

        elseif ENABLE_UWB && UWB_SOURCE == "ROS"
            [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
                receiveAndProcessUwbRos(uwb_ros, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb);
        end

        % ==========================================
        % B. OptiTrack UDP Reception
        % ==========================================
        [latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti] = ...
            receiveAndProcessOpti(u_opti, t_exp, latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti);

        % ==========================================
        % C. Synchronous ROS Publishing and Robot Control (30 Hz)
        % ==========================================
        if toc(t_run) > T_run
            t_run = tic;

            % Get latest accelerometer data if the selected robot exposes it.
            if ENABLE_ACCEL_READ && ROS_INITIALIZED && ~isempty(robot)
                if SelectedRobot == "Limu"
                    try
                        current_accel = robot.latest_accel;
                    catch
                        current_accel = [0, 0, 0];
                    end
                elseif SelectedRobot == "Bebop2"
                    % Future Bebop2 accelerometer implementation
                elseif SelectedRobot == "Crazyflie"
                    % Future Crazyflie accelerometer implementation
                end

                print_counter_accel = print_counter_accel + 1;
                if mod(print_counter_accel, 200) == 0
                    fprintf('[ROBOT] Accel -> X: %.2f, Y: %.2f, Z: %.2f\n', current_accel(1), current_accel(2), current_accel(3));
                end
            end

            % Publish filtered UWB and ground-truth positions to ROS when enabled.
            if ENABLE_ROS && ROS_INITIALIZED
                publishPointIfAvailable(ros_publishers, 'uwb', latest_uwb_filt);
                publishPointIfAvailable(ros_publishers, 'gt', latest_opti);
            end

            % Robot control uses filtered UWB and OptiTrack data for now, same as the old script.
            if ENABLE_ROBOT_CONTROL && ROS_INITIALIZED && ~isempty(robot)
                try
                    robot.update(latest_uwb_filt, latest_opti);
                catch ME
                    fprintf('[MASTER] Robot update error: %s\n', ME.message);
                    ENABLE_ROBOT_CONTROL = false;
                end

                if toc(t_exp) > movement_time && robot_is_moving
                    disp('[MASTER] Movement Time Finished. Initiating Stop Sequence.');
                    robot.landAndStop();
                    robot_is_moving = false;
                    ENABLE_ROBOT_CONTROL = false;
                end

                try
                    if robot.EmergencyTriggered
                        disp('################ EMERGENCY ################');
                        break;
                    end
                catch
                    % Some robot classes may not have EmergencyTriggered yet.
                end
            end

            drawnow;
        end

        pause(0.001);
    end

catch ME
    fprintf('[MASTER] CRASH: %s\n', ME.message);
end


%% 11. SAFE SHUTDOWN
% =====================================================================================================================
disp('[MASTER] Cleaning up resources...');

if ~isempty(robot) && robot_is_moving
    try
        robot.landAndStop();
    catch ME
        fprintf('[MASTER] Robot stop error: %s\n', ME.message);
    end
end

if fid > 0
    closeFileSafe(fid);
end

try
    clear u_settings u_uwb u_opti;
catch
end

try
    rosshutdown;
catch
end

disp('[MASTER] Shutdown Complete.');

end


%% ====================================================================================================================
% LOCAL HELPER FUNCTIONS
% =====================================================================================================================

% Reads the latest available UDP text packet from a udpport object.
% ---------------------------------------------------------------------------------------------------------------------
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


% Parses the JSON settings packet from MasterControlStation.
% ---------------------------------------------------------------------------------------------------------------------
function [settings, valid] = parseSettingsPacket(packet)
    settings = struct();
    valid = false;

    try
        decoded = jsondecode(char(packet));

        if isfield(decoded, 'packet_type')
            packet_type = string(decoded.packet_type);
            if packet_type == "settings"
                settings = decoded;
                valid = true;
            end
        end

    catch ME
        fprintf('[MASTER] Settings JSON parse error: %s\n', ME.message);
    end
end


% Reads a field from a struct. If the field does not exist, returns a default value.
% ---------------------------------------------------------------------------------------------------------------------
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


% Converts Python/JSON/MATLAB values to logical true/false.
% ---------------------------------------------------------------------------------------------------------------------
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


% Parses the UWB-only UDP packet.
% Format: timestamp,x,y,z,quality,listener_id,network_id,position_type
% ---------------------------------------------------------------------------------------------------------------------
function uwb = parseUwbPacket(packet)
    uwb = struct();
    uwb.valid = false;
    uwb.timestamp = NaN;
    uwb.position = [NaN, NaN, NaN];
    uwb.quality = NaN;
    uwb.listener_id = NaN;
    uwb.network_id = NaN;
    uwb.position_type = "unknown";

    try
        parts = strsplit(char(strtrim(packet)), ',');

        if numel(parts) < 8
            return;
        end

        uwb.timestamp = str2double(parts{1});
        x = str2double(parts{2});
        y = str2double(parts{3});
        z = str2double(parts{4});
        uwb.quality = str2double(parts{5});
        uwb.listener_id = str2double(parts{6});
        uwb.network_id = str2double(parts{7});
        uwb.position_type = string(strtrim(parts{8}));

        if any(isnan([uwb.timestamp, x, y, z]))
            return;
        end

        uwb.position = [x, y, z];
        uwb.valid = true;

    catch
        uwb.valid = false;
    end
end


% Parses the OptiTrack-only UDP packet.
% Format: timestamp,x,y,z,quality,rigid_body_id,source_type
% ---------------------------------------------------------------------------------------------------------------------
function opti = parseOptiPacket(packet)
    opti = struct();
    opti.valid = false;
    opti.timestamp = NaN;
    opti.position = [NaN, NaN, NaN];
    opti.quality = NaN;
    opti.rigid_body_id = NaN;
    opti.source_type = "unknown";

    try
        parts = strsplit(char(strtrim(packet)), ',');

        if numel(parts) < 7
            return;
        end

        opti.timestamp = str2double(parts{1});
        x = str2double(parts{2});
        y = str2double(parts{3});
        z = str2double(parts{4});
        opti.quality = str2double(parts{5});
        opti.rigid_body_id = str2double(parts{6});
        opti.source_type = string(strtrim(parts{7}));

        if any(isnan([opti.timestamp, x, y, z]))
            return;
        end

        opti.position = [x, y, z];
        opti.valid = true;

    catch
        opti.valid = false;
    end
end


% Receives UWB data, filters it and writes MATLAB-filtered UWB to CSV.
% ---------------------------------------------------------------------------------------------------------------------
function [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
    receiveAndProcessUwb(u_uwb, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb)

    packet = readLatestUdpPacket(u_uwb);

    if strlength(packet) == 0
        return;
    end

    uwb = parseUwbPacket(packet);

    if ~uwb.valid
        fprintf('[MATLAB] Invalid UWB packet: %s\n', packet);
        return;
    end

    latest_uwb_raw = uwb.position;
    latest_uwb_time = uwb.timestamp;
    latest_uwb_quality = uwb.quality;
    latest_uwb_position_type = uwb.position_type;
    last_uwb_receive_time = toc(t_exp);

    [fx, fy, fz] = uwb_filter.process(uwb.position(1), uwb.position(2), uwb.position(3));
    latest_uwb_filt = [fx, fy, fz];

    if fid > 0
        fprintf(fid, '%.3f,%.4f,%.4f,%.4f\n', latest_uwb_time, fx, fy, fz);
    end

    print_counter_uwb = print_counter_uwb + 1;
    if mod(print_counter_uwb, 10) == 0
        fprintf('[MATLAB] UWB Filtered -> X: %.2f, Y: %.2f, Z: %.2f | Quality: %.2f | Type: %s\n', ...
            fx, fy, fz, latest_uwb_quality, latest_uwb_position_type);
    end
end


% Creates the future UWB-over-ROS subscriber.
% The callback stores the newest message in a small persistent message store.
% ---------------------------------------------------------------------------------------------------------------------
function uwb_ros = setupUwbRosInput(topic_name, msg_type)
    uwb_ros = struct();
    uwb_ros.enabled = false;
    uwb_ros.subscriber = [];
    uwb_ros.topic = topic_name;
    uwb_ros.msg_type = msg_type;

    try
        fprintf('[MASTER] Creating UWB ROS subscriber: %s | %s\n', topic_name, msg_type);

        % Clear old stored message.
        uwbRosMessageStore("clear");

        uwb_ros.subscriber = rossubscriber( ...
            char(topic_name), ...
            char(msg_type), ...
            @uwbRosCallback, ...
            "DataFormat", "struct");

        uwb_ros.enabled = true;
        disp('[MASTER] UWB ROS subscriber created.');

    catch ME
        fprintf('[MASTER] Warning: Could not create UWB ROS subscriber: %s\n', ME.message);
        uwb_ros.enabled = false;
    end
end


% Callback for future UWB-over-ROS input.
% ---------------------------------------------------------------------------------------------------------------------
function uwbRosCallback(~, msg)
    uwbRosMessageStore("set", msg);
end


% Small persistent store for the latest UWB ROS message.
% This avoids blocking the main loop.
% ---------------------------------------------------------------------------------------------------------------------
function [msg, has_new] = uwbRosMessageStore(action, new_msg)
    persistent latest_msg latest_is_new

    if isempty(latest_is_new)
        latest_is_new = false;
        latest_msg = [];
    end

    msg = [];
    has_new = false;

    if action == "set"
        latest_msg = new_msg;
        latest_is_new = true;

    elseif action == "get"
        if latest_is_new
            msg = latest_msg;
            has_new = true;
            latest_is_new = false;
        end

    elseif action == "clear"
        latest_msg = [];
        latest_is_new = false;
    end
end


% Receives UWB data from ROS, filters it and writes MATLAB-filtered UWB to CSV.
% This is a placeholder input path for future custom PCB / ROS-based UWB.
%
% Expected default input:
%   Topic: /uwb/raw_position
%   Type:  geometry_msgs/PointStamped
%
% Also accepts:
%   geometry_msgs/Point
%   geometry_msgs/PoseStamped
% ---------------------------------------------------------------------------------------------------------------------
function [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
    receiveAndProcessUwbRos(uwb_ros, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb)

    if ~isfield(uwb_ros, 'enabled') || ~uwb_ros.enabled
        return;
    end

    [msg, has_new] = uwbRosMessageStore("get");

    if ~has_new
        return;
    end

    [valid, position, message_time, position_type] = parseUwbRosMessage(msg, toc(t_exp));

    if ~valid
        disp('[MATLAB] Invalid UWB ROS message.');
        return;
    end

    latest_uwb_raw = position;
    latest_uwb_time = message_time;
    latest_uwb_quality = NaN;
    latest_uwb_position_type = position_type;
    last_uwb_receive_time = toc(t_exp);

    [fx, fy, fz] = uwb_filter.process(position(1), position(2), position(3));
    latest_uwb_filt = [fx, fy, fz];

    if fid > 0
        fprintf(fid, '%.3f,%.4f,%.4f,%.4f\n', latest_uwb_time, fx, fy, fz);
    end

    print_counter_uwb = print_counter_uwb + 1;

    if mod(print_counter_uwb, 10) == 0
        fprintf('[MATLAB] UWB ROS Filtered -> X: %.2f, Y: %.2f, Z: %.2f | Type: %s\n', ...
            fx, fy, fz, latest_uwb_position_type);
    end
end


% Parses possible future UWB ROS message types.
% Supports:
%   geometry_msgs/Point
%   geometry_msgs/PointStamped
%   geometry_msgs/PoseStamped
% ---------------------------------------------------------------------------------------------------------------------
function [valid, position, message_time, position_type] = parseUwbRosMessage(msg, fallback_time)
    valid = false;
    position = [NaN, NaN, NaN];
    message_time = fallback_time;
    position_type = "uwb_ros_unknown";

    try
        % geometry_msgs/PointStamped
        if isfield(msg, 'Point')
            position = [double(msg.Point.X), double(msg.Point.Y), double(msg.Point.Z)];
            position_type = "uwb_ros_pointstamped";

        % geometry_msgs/PoseStamped
        elseif isfield(msg, 'Pose') && isfield(msg.Pose, 'Position')
            position = [double(msg.Pose.Position.X), double(msg.Pose.Position.Y), double(msg.Pose.Position.Z)];
            position_type = "uwb_ros_posestamped";

        % geometry_msgs/Point
        elseif isfield(msg, 'X') && isfield(msg, 'Y') && isfield(msg, 'Z')
            position = [double(msg.X), double(msg.Y), double(msg.Z)];
            position_type = "uwb_ros_point";

        else
            return;
        end

        if any(isnan(position)) || any(isinf(position))
            return;
        end

        % Use ROS message timestamp if it exists.
        if isfield(msg, 'Header') && isfield(msg.Header, 'Stamp')
            stamp = msg.Header.Stamp;

            if isfield(stamp, 'Sec') && isfield(stamp, 'Nsec')
                message_time = double(stamp.Sec) + double(stamp.Nsec) * 1e-9;
            end
        end

        valid = true;

    catch
        valid = false;
    end
end


% Receives OptiTrack data.
% ---------------------------------------------------------------------------------------------------------------------
function [latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti] = ...
    receiveAndProcessOpti(u_opti, t_exp, latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti)

    packet = readLatestUdpPacket(u_opti);

    if strlength(packet) == 0
        return;
    end

    opti = parseOptiPacket(packet);

    if ~opti.valid
        fprintf('[MATLAB] Invalid OptiTrack packet: %s\n', packet);
        return;
    end

    latest_opti = opti.position;
    latest_opti_time = opti.timestamp;
    latest_opti_quality = opti.quality;
    last_opti_receive_time = toc(t_exp);

    print_counter_opti = print_counter_opti + 1;
    if mod(print_counter_opti, 200) == 0
        fprintf('[MATLAB] OptiTrack Packet -> X: %.2f, Y: %.2f, Z: %.2f | Quality: %.2f\n', ...
            latest_opti(1), latest_opti(2), latest_opti(3), latest_opti_quality);
    end
end


% Checks if a sensor has recently produced data.
% ---------------------------------------------------------------------------------------------------------------------
function is_fresh = isSensorFresh(last_receive_time, current_time, freshness_timeout)
    is_fresh = isfinite(last_receive_time) && ((current_time - last_receive_time) <= freshness_timeout);
end


% Publishes a geometry_msgs/Point if the requested publisher exists and the position is valid.
% ---------------------------------------------------------------------------------------------------------------------
function publishPointIfAvailable(ros_publishers, publisher_name, position)
    if any(isnan(position))
        return;
    end

    try
        if isfield(ros_publishers, publisher_name)
            pub = ros_publishers.(publisher_name);
            msg = ros_publishers.([publisher_name, '_msg']);

            msg.X = position(1);
            msg.Y = position(2);
            msg.Z = position(3);

            send(pub, msg);
        end
    catch ME
        fprintf('[ROS] Publish error on %s: %s\n', publisher_name, ME.message);
    end
end


% Placeholder for future GPS RTK ROS input handled inside MATLAB.
% ---------------------------------------------------------------------------------------------------------------------
function startGpsRtkPlaceholder(topic_name, frame_name)
    fprintf('[MASTER] GPS RTK selected. Placeholder topic: %s | frame: %s\n', topic_name, frame_name);
    disp('[MASTER] GPS RTK ROS reading is a placeholder in this MATLAB script for now.');
end


% Closes a file ID safely.
% ---------------------------------------------------------------------------------------------------------------------
function closeFileSafe(fid)
    try
        if fid > 0
            fclose(fid);
        end
    catch
    end
end


% 
% 
% % =========================================================================
% % MATLAB MASTER UWB CONTROL
% % Author: Renzo Eisma
% % Date: 09/04/2026
% %
% % Description:
% % Master control script for receiving settings, UWB data and ground-truth
% % data from the Python software framework. The script filters the incoming
% % UWB position, logs the MATLAB-filtered UWB data, optionally publishes
% % position data to ROS and optionally sends robot control commands.
% %
% % New framework:
% %   Port 5004 = Settings/config packet from MasterControlStation.py
% %   Port 5005 = Live UWB packets from uwb_sensor.py
% %   Port 5006 = Live OptiTrack packets from NatNetClient.py
% % =========================================================================
% 
% function MatlabMasterUWBControl()
% 
% %% 0. SYSTEM CLEANUP & INIT
% % =====================================================================================================================
% clear classes;
% close all;
% clc;
% 
% try
%     rosshutdown;
% catch
% end
% pause(0.5); % Ensure old ROS connections are killed immediately to clear ghost nodes
% 
% 
% %% 1. MANUAL SYSTEM CONFIGURATION
% % =====================================================================================================================
% % Robot selection is intentionally kept manual for now.
% SelectedRobot = "Bebop2";      % "Bebop2", "Crazyflie", "Limu"
% 
% % ROS master IP is also kept manual for now.
% ros_ip = '192.168.0.100';
% 
% % General timing settings
% T_exp = 2400;                  % Maximum runtime [s]
% T_run = 1/30;                  % Robot/control update rate [s]
% movement_time = 10;            % Safety movement time before stop [s]
% SENSOR_WAIT_TIMEOUT = 30;      % Maximum time to wait for required sensors [s]
% SENSOR_FRESH_TIMEOUT = 2.0;    % Sensor is considered alive if last packet is newer than this [s]
% 
% % Accelerometer reading is only used through robot classes in MATLAB.
% ENABLE_ACCEL_READ = true;
% 
% 
% %% 2. UDP PORT SETUP
% % =====================================================================================================================
% disp('[MASTER] Opening UDP Ports...');
% 
% try
%     u_settings = udpport("LocalHost", "127.0.0.1", "LocalPort", 5004);
%     u_uwb      = udpport("LocalHost", "127.0.0.1", "LocalPort", 5005);
%     u_opti     = udpport("LocalHost", "127.0.0.1", "LocalPort", 5006);
% catch ME
%     fprintf('[MASTER] Failed to open UDP ports: %s\n', ME.message);
%     disp('[MASTER] Make sure no old MATLAB process is still using ports 5004, 5005 or 5006.');
%     return;
% end
% 
% fprintf('[MASTER] Settings UDP: 127.0.0.1:%d\n', 5004);
% fprintf('[MASTER] UWB UDP:      127.0.0.1:%d\n', 5005);
% fprintf('[MASTER] Opti UDP:     127.0.0.1:%d\n', 5006);
% 
% 
% %% 3. WAIT FOR SETTINGS PACKET FROM MASTERCONTROLSTATION
% % =====================================================================================================================
% disp('[MASTER] Waiting for settings packet on port 5004...');
% 
% settings = struct();
% settings_received = false;
% 
% while ~settings_received
%     packet = readLatestUdpPacket(u_settings);
% 
%     if strlength(packet) > 0
%         [settings, settings_received] = parseSettingsPacket(packet);
% 
%         if settings_received
%             disp('[MASTER] Settings packet received.');
%         else
%             disp('[MASTER] Received invalid settings packet. Waiting for a new one...');
%         end
%     end
% 
%     pause(0.05);
% end
% 
% 
% %% 4. APPLY SETTINGS FROM PYTHON
% % =====================================================================================================================
% % The Python GUI decides what is enabled. MATLAB uses these settings to decide
% % which sensors to wait for and whether ROS / robot control should start.
% 
% uwb_settings = getFieldDefault(settings, 'uwb', struct());
% gt_settings = getFieldDefault(settings, 'ground_truth', struct());
% routing_settings = getFieldDefault(settings, 'routing', struct());
% 
% session_name = string(getFieldDefault(settings, 'session_name', "Session_Unknown"));
% session_dir = string(getFieldDefault(settings, 'session_dir', pwd));
% 
% ENABLE_UWB = toLogical(getFieldDefault(uwb_settings, 'enabled', false));
% UWB_SOURCE = string(getFieldDefault(uwb_settings, 'source', "Listener"));
% UWB_READ_TYPE = string(getFieldDefault(uwb_settings, 'read_type', "Tag Position"));
% UWB_NETWORK_SCALE = string(getFieldDefault(uwb_settings, 'network_scale', "1 Network / 1 Listener"));
% 
% ENABLE_GT = toLogical(getFieldDefault(gt_settings, 'enabled', false));
% GT_SOURCE = string(getFieldDefault(gt_settings, 'type', "OptiTrack"));
% GPS_RTK_TOPIC = string(getFieldDefault(gt_settings, 'gps_rtk_topic', "/gps/rtk"));
% GPS_RTK_FRAME = string(getFieldDefault(gt_settings, 'gps_rtk_frame', "map"));
% 
% ENABLE_ROS = toLogical(getFieldDefault(routing_settings, 'send_data_to_ros', false));
% ENABLE_ROBOT_CONTROL = toLogical(getFieldDefault(routing_settings, 'control_robots', false));
% 
% if ENABLE_ROBOT_CONTROL && ~ENABLE_ROS
%     disp('[MASTER] Warning: Control Robots was enabled, but Send Data to ROS was disabled.');
%     disp('[MASTER] Robot control requires ROS, so robot control is disabled.');
%     ENABLE_ROBOT_CONTROL = false;
% end
% 
% if ~isfolder(session_dir)
%     mkdir(session_dir);
% end
% 
% fprintf('\n[MASTER] Session name: %s\n', session_name);
% fprintf('[MASTER] Session folder: %s\n', session_dir);
% fprintf('[MASTER] UWB enabled: %d | Source: %s | Read mode: %s | Network scale: %s\n', ...
%     ENABLE_UWB, UWB_SOURCE, UWB_READ_TYPE, UWB_NETWORK_SCALE);
% fprintf('[MASTER] Ground truth enabled: %d | Source: %s\n', ENABLE_GT, GT_SOURCE);
% fprintf('[MASTER] ROS enabled: %d | Robot control enabled: %d\n\n', ENABLE_ROS, ENABLE_ROBOT_CONTROL);
% 
% 
% %% 5. MATLAB FILTER AND FILE LOGGING SETUP
% % =====================================================================================================================
% uwb_filter = FilterUWB(0.1);
% fid = -1;
% fileCleaner = [];
% 
% if ENABLE_UWB
%     csv_name = "[Log]_uwbFilteredMatlab_listener1_" + session_name + ".csv";
%     log_filename = fullfile(session_dir, csv_name);
% 
%     fid = fopen(log_filename, 'w');
% 
%     if fid ~= -1
%         fprintf(fid, 'Time,Filt_X,Filt_Y,Filt_Z\n');
%         disp(['[MASTER] MATLAB filtered UWB log initialized: ', char(log_filename)]);
%         fileCleaner = onCleanup(@() closeFileSafe(fid));
%     else
%         disp('[MASTER] Warning: Could not open MATLAB filtered UWB CSV. Logging disabled.');
%     end
% else
%     disp('[MASTER] UWB disabled. MATLAB filtered UWB CSV will not be created.');
% end
% 
% 
% %% 6. ROS / ROBOT SETUP
% % =====================================================================================================================
% robot = [];
% ROS_INITIALIZED = false;
% robot_is_moving = false;
% current_accel = [0, 0, 0];
% ros_publishers = struct();
% 
% if ENABLE_ROS
%     disp('[MASTER] Initializing ROS Network...');
% 
%     try
%         try
%             fclose(instrfindall);
%         catch
%         end
% 
%         rosshutdown;
%         pause(1);
%         rosinit(ros_ip);
%         ROS_INITIALIZED = true;
% 
%         fprintf('[MASTER] ROS initialized. Selected Robot: %s\n', SelectedRobot);
% 
%         % Position publishers are intentionally simple. They are useful for debugging and later ROS integration.
%         try
%             ros_publishers.uwb = rospublisher('/uwb/filtered_position', 'geometry_msgs/Point');
%             ros_publishers.uwb_msg = rosmessage(ros_publishers.uwb);
% 
%             ros_publishers.gt = rospublisher('/ground_truth/position', 'geometry_msgs/Point');
%             ros_publishers.gt_msg = rosmessage(ros_publishers.gt);
% 
%             disp('[MASTER] ROS position publishers created.');
%         catch ME
%             fprintf('[MASTER] Warning: Could not create ROS publishers: %s\n', ME.message);
%         end
% 
%         % Robot class is only initialized when robot control is enabled.
%         if ENABLE_ROBOT_CONTROL
%             if SelectedRobot == "Limu"
%                 robot = RobotControlCodeLimuClass();
%             elseif SelectedRobot == "Bebop2"
%                 robot = RobotControlCodeBebopClass();
%             elseif SelectedRobot == "Crazyflie"
%                 robot = RobotControlCodeLimuClass(); % Placeholder until Crazyflie class exists
%             end
%         end
% 
%     catch ME
%         fprintf('[MASTER] ROS Error: %s\n', ME.message);
%         ENABLE_ROS = false;
%         ENABLE_ROBOT_CONTROL = false;
%         ENABLE_ACCEL_READ = false;
%         ROS_INITIALIZED = false;
%     end
% end
% 
% 
% %% 7. MATLAB/ROS SENSOR PLACEHOLDERS
% % =====================================================================================================================
% % Python currently live-logs UWB listener data and OptiTrack data.
% % GPS RTK and custom PCB UWB-over-ROS are intended to be handled in MATLAB/ROS later.
% 
% if ENABLE_UWB && UWB_SOURCE == "ROS"
%     startUwbRosPlaceholder();
% end
% 
% if ENABLE_GT && GT_SOURCE == "GPS RTK"
%     startGpsRtkPlaceholder(GPS_RTK_TOPIC, GPS_RTK_FRAME);
% end
% 
% 
% %% 8. MAIN LOOP VARIABLES
% % =====================================================================================================================
% latest_uwb_raw = [NaN, NaN, NaN];
% latest_uwb_filt = [NaN, NaN, NaN];
% latest_uwb_time = NaN;
% latest_uwb_quality = NaN;
% latest_uwb_position_type = "none";
% last_uwb_receive_time = -Inf;
% 
% latest_opti = [NaN, NaN, NaN];
% latest_opti_time = NaN;
% latest_opti_quality = NaN;
% last_opti_receive_time = -Inf;
% 
% print_counter_uwb = 0;
% print_counter_opti = 0;
% print_counter_accel = 0;
% 
% require_uwb_udp = ENABLE_UWB && UWB_SOURCE == "Listener";
% require_opti_udp = ENABLE_GT && GT_SOURCE == "OptiTrack";
% 
% if ENABLE_UWB && UWB_SOURCE ~= "Listener"
%     disp('[MASTER] UWB source is not Listener. UWB UDP waiting is skipped for now.');
% end
% 
% if ENABLE_GT && GT_SOURCE ~= "OptiTrack"
%     disp('[MASTER] Ground-truth source is not OptiTrack. OptiTrack UDP waiting is skipped for now.');
% end
% 
% 
% %% 9. WAIT FOR REQUIRED SENSORS TO BECOME ACTIVE
% % =====================================================================================================================
% disp('[MASTER] Waiting for required sensors to become active...');
% 
% sensor_wait_start = tic;
% t_exp = tic;
% 
% while toc(sensor_wait_start) < SENSOR_WAIT_TIMEOUT
%     % Receive data while waiting, because Python may already be streaming.
%     [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
%         receiveAndProcessUwb(u_uwb, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb);
% 
%     [latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti] = ...
%         receiveAndProcessOpti(u_opti, t_exp, latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti);
% 
%     uwb_ready = ~require_uwb_udp || isSensorFresh(last_uwb_receive_time, toc(t_exp), SENSOR_FRESH_TIMEOUT);
%     opti_ready = ~require_opti_udp || isSensorFresh(last_opti_receive_time, toc(t_exp), SENSOR_FRESH_TIMEOUT);
% 
%     if uwb_ready && opti_ready
%         disp('[MASTER] Required sensors are active.');
%         break;
%     end
% 
%     if require_uwb_udp && ~uwb_ready
%         fprintf('[MASTER] Waiting for UWB listener data on port 5005...\n');
%     end
% 
%     if require_opti_udp && ~opti_ready
%         fprintf('[MASTER] Waiting for OptiTrack data on port 5006...\n');
%     end
% 
%     pause(0.5);
% end
% 
% uwb_ready = ~require_uwb_udp || isSensorFresh(last_uwb_receive_time, toc(t_exp), SENSOR_FRESH_TIMEOUT);
% opti_ready = ~require_opti_udp || isSensorFresh(last_opti_receive_time, toc(t_exp), SENSOR_FRESH_TIMEOUT);
% 
% if ~(uwb_ready && opti_ready)
%     disp('[MASTER] Warning: Not all required sensors became active before timeout.');
%     disp('[MASTER] Logging/filtering will continue, but robot control is disabled for safety.');
%     ENABLE_ROBOT_CONTROL = false;
% else
%     disp('[MASTER] Sensor startup check complete.');
% end
% 
% % Start the robot movement only after sensors are confirmed and ROS is ready.
% if ENABLE_ROBOT_CONTROL && ROS_INITIALIZED && ~isempty(robot)
%     try
%         robot.perform_movement();
%         robot_is_moving = true;
%         disp('[MASTER] Robot movement started.');
%     catch ME
%         fprintf('[MASTER] Could not start robot movement: %s\n', ME.message);
%         ENABLE_ROBOT_CONTROL = false;
%         robot_is_moving = false;
%     end
% elseif ENABLE_ROS && ROS_INITIALIZED
%     disp('[MASTER] ROS is enabled, but robot movement is disabled.');
% end
% 
% 
% %% 10. HIGH SPEED EXECUTION LOOP
% % =====================================================================================================================
% disp('[MASTER] Entering main MATLAB control loop.');
% 
% t_run = tic;
% 
% try
%     while toc(t_exp) < T_exp
%         % ==========================================
%         % A. UWB UDP Reception + Filtering + Logging
%         % ==========================================
%         [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
%             receiveAndProcessUwb(u_uwb, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb);
% 
%         % ==========================================
%         % B. OptiTrack UDP Reception
%         % ==========================================
%         [latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti] = ...
%             receiveAndProcessOpti(u_opti, t_exp, latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti);
% 
%         % ==========================================
%         % C. Synchronous ROS Publishing and Robot Control (30 Hz)
%         % ==========================================
%         if toc(t_run) > T_run
%             t_run = tic;
% 
%             % Get latest accelerometer data if the selected robot exposes it.
%             if ENABLE_ACCEL_READ && ROS_INITIALIZED && ~isempty(robot)
%                 if SelectedRobot == "Limu"
%                     try
%                         current_accel = robot.latest_accel;
%                     catch
%                         current_accel = [0, 0, 0];
%                     end
%                 elseif SelectedRobot == "Bebop2"
%                     % Future Bebop2 accelerometer implementation
%                 elseif SelectedRobot == "Crazyflie"
%                     % Future Crazyflie accelerometer implementation
%                 end
% 
%                 print_counter_accel = print_counter_accel + 1;
%                 if mod(print_counter_accel, 200) == 0
%                     fprintf('[ROBOT] Accel -> X: %.2f, Y: %.2f, Z: %.2f\n', current_accel(1), current_accel(2), current_accel(3));
%                 end
%             end
% 
%             % Publish filtered UWB and ground-truth positions to ROS when enabled.
%             if ENABLE_ROS && ROS_INITIALIZED
%                 publishPointIfAvailable(ros_publishers, 'uwb', latest_uwb_filt);
%                 publishPointIfAvailable(ros_publishers, 'gt', latest_opti);
%             end
% 
%             % Robot control uses filtered UWB and OptiTrack data for now, same as the old script.
%             if ENABLE_ROBOT_CONTROL && ROS_INITIALIZED && ~isempty(robot)
%                 try
%                     robot.update(latest_uwb_filt, latest_opti);
%                 catch ME
%                     fprintf('[MASTER] Robot update error: %s\n', ME.message);
%                     ENABLE_ROBOT_CONTROL = false;
%                 end
% 
%                 if toc(t_exp) > movement_time && robot_is_moving
%                     disp('[MASTER] Movement Time Finished. Initiating Stop Sequence.');
%                     robot.landAndStop();
%                     robot_is_moving = false;
%                     ENABLE_ROBOT_CONTROL = false;
%                 end
% 
%                 try
%                     if robot.EmergencyTriggered
%                         disp('################ EMERGENCY ################');
%                         break;
%                     end
%                 catch
%                     % Some robot classes may not have EmergencyTriggered yet.
%                 end
%             end
% 
%             drawnow;
%         end
% 
%         pause(0.001);
%     end
% 
% catch ME
%     fprintf('[MASTER] CRASH: %s\n', ME.message);
% end
% 
% 
% %% 11. SAFE SHUTDOWN
% % =====================================================================================================================
% disp('[MASTER] Cleaning up resources...');
% 
% if ~isempty(robot) && robot_is_moving
%     try
%         robot.landAndStop();
%     catch ME
%         fprintf('[MASTER] Robot stop error: %s\n', ME.message);
%     end
% end
% 
% if fid > 0
%     closeFileSafe(fid);
% end
% 
% try
%     clear u_settings u_uwb u_opti;
% catch
% end
% 
% try
%     rosshutdown;
% catch
% end
% 
% disp('[MASTER] Shutdown Complete.');
% 
% end
% 
% 
% %% ====================================================================================================================
% % LOCAL HELPER FUNCTIONS
% % =====================================================================================================================
% 
% % Reads the latest available UDP text packet from a udpport object.
% % ---------------------------------------------------------------------------------------------------------------------
% function packet = readLatestUdpPacket(u)
%     packet = "";
% 
%     try
%         if u.NumBytesAvailable <= 0
%             return;
%         end
% 
%         raw_data = read(u, u.NumBytesAvailable, "string");
% 
%         if isstring(raw_data)
%             if numel(raw_data) > 1
%                 packet = raw_data(end);
%             else
%                 packet = raw_data;
%             end
%         elseif ischar(raw_data)
%             packet = string(raw_data);
%         else
%             packet = string(char(raw_data));
%         end
% 
%         packet = strtrim(packet);
% 
%         % If multiple lines are received, use the latest non-empty one.
%         if contains(packet, newline)
%             parts = splitlines(packet);
%             parts = parts(strlength(strtrim(parts)) > 0);
%             if ~isempty(parts)
%                 packet = strtrim(parts(end));
%             end
%         end
% 
%     catch ME
%         fprintf('[UDP] Read error: %s\n', ME.message);
%         packet = "";
%     end
% end
% 
% 
% % Parses the JSON settings packet from MasterControlStation.
% % ---------------------------------------------------------------------------------------------------------------------
% function [settings, valid] = parseSettingsPacket(packet)
%     settings = struct();
%     valid = false;
% 
%     try
%         decoded = jsondecode(char(packet));
% 
%         if isfield(decoded, 'packet_type')
%             packet_type = string(decoded.packet_type);
%             if packet_type == "settings"
%                 settings = decoded;
%                 valid = true;
%             end
%         end
% 
%     catch ME
%         fprintf('[MASTER] Settings JSON parse error: %s\n', ME.message);
%     end
% end
% 
% 
% % Reads a field from a struct. If the field does not exist, returns a default value.
% % ---------------------------------------------------------------------------------------------------------------------
% function value = getFieldDefault(s, field_name, default_value)
%     if isstruct(s) && isfield(s, field_name)
%         value = s.(field_name);
%         if isempty(value)
%             value = default_value;
%         end
%     else
%         value = default_value;
%     end
% end
% 
% 
% % Converts Python/JSON/MATLAB values to logical true/false.
% % ---------------------------------------------------------------------------------------------------------------------
% function value = toLogical(input_value)
%     if islogical(input_value)
%         value = input_value;
%     elseif isnumeric(input_value)
%         value = input_value ~= 0;
%     elseif ischar(input_value) || isstring(input_value)
%         text_value = lower(strtrim(string(input_value)));
%         value = any(text_value == ["true", "1", "yes", "on"]);
%     else
%         value = false;
%     end
% end
% 
% 
% % Parses the UWB-only UDP packet.
% % Format: timestamp,x,y,z,quality,listener_id,network_id,position_type
% % ---------------------------------------------------------------------------------------------------------------------
% function uwb = parseUwbPacket(packet)
%     uwb = struct();
%     uwb.valid = false;
%     uwb.timestamp = NaN;
%     uwb.position = [NaN, NaN, NaN];
%     uwb.quality = NaN;
%     uwb.listener_id = NaN;
%     uwb.network_id = NaN;
%     uwb.position_type = "unknown";
% 
%     try
%         parts = strsplit(char(strtrim(packet)), ',');
% 
%         if numel(parts) < 8
%             return;
%         end
% 
%         uwb.timestamp = str2double(parts{1});
%         x = str2double(parts{2});
%         y = str2double(parts{3});
%         z = str2double(parts{4});
%         uwb.quality = str2double(parts{5});
%         uwb.listener_id = str2double(parts{6});
%         uwb.network_id = str2double(parts{7});
%         uwb.position_type = string(strtrim(parts{8}));
% 
%         if any(isnan([uwb.timestamp, x, y, z]))
%             return;
%         end
% 
%         uwb.position = [x, y, z];
%         uwb.valid = true;
% 
%     catch
%         uwb.valid = false;
%     end
% end
% 
% 
% % Parses the OptiTrack-only UDP packet.
% % Format: timestamp,x,y,z,quality,rigid_body_id,source_type
% % ---------------------------------------------------------------------------------------------------------------------
% function opti = parseOptiPacket(packet)
%     opti = struct();
%     opti.valid = false;
%     opti.timestamp = NaN;
%     opti.position = [NaN, NaN, NaN];
%     opti.quality = NaN;
%     opti.rigid_body_id = NaN;
%     opti.source_type = "unknown";
% 
%     try
%         parts = strsplit(char(strtrim(packet)), ',');
% 
%         if numel(parts) < 7
%             return;
%         end
% 
%         opti.timestamp = str2double(parts{1});
%         x = str2double(parts{2});
%         y = str2double(parts{3});
%         z = str2double(parts{4});
%         opti.quality = str2double(parts{5});
%         opti.rigid_body_id = str2double(parts{6});
%         opti.source_type = string(strtrim(parts{7}));
% 
%         if any(isnan([opti.timestamp, x, y, z]))
%             return;
%         end
% 
%         opti.position = [x, y, z];
%         opti.valid = true;
% 
%     catch
%         opti.valid = false;
%     end
% end
% 
% 
% % Receives UWB data, filters it and writes MATLAB-filtered UWB to CSV.
% % ---------------------------------------------------------------------------------------------------------------------
% function [latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb] = ...
%     receiveAndProcessUwb(u_uwb, uwb_filter, fid, t_exp, latest_uwb_raw, latest_uwb_filt, latest_uwb_time, latest_uwb_quality, latest_uwb_position_type, last_uwb_receive_time, print_counter_uwb)
% 
%     packet = readLatestUdpPacket(u_uwb);
% 
%     if strlength(packet) == 0
%         return;
%     end
% 
%     uwb = parseUwbPacket(packet);
% 
%     if ~uwb.valid
%         fprintf('[MATLAB] Invalid UWB packet: %s\n', packet);
%         return;
%     end
% 
%     latest_uwb_raw = uwb.position;
%     latest_uwb_time = uwb.timestamp;
%     latest_uwb_quality = uwb.quality;
%     latest_uwb_position_type = uwb.position_type;
%     last_uwb_receive_time = toc(t_exp);
% 
%     [fx, fy, fz] = uwb_filter.process(uwb.position(1), uwb.position(2), uwb.position(3));
%     latest_uwb_filt = [fx, fy, fz];
% 
%     if fid > 0
%         fprintf(fid, '%.3f,%.4f,%.4f,%.4f\n', latest_uwb_time, fx, fy, fz);
%     end
% 
%     print_counter_uwb = print_counter_uwb + 1;
%     if mod(print_counter_uwb, 10) == 0
%         fprintf('[MATLAB] UWB Filtered -> X: %.2f, Y: %.2f, Z: %.2f | Quality: %.2f | Type: %s\n', ...
%             fx, fy, fz, latest_uwb_quality, latest_uwb_position_type);
%     end
% end
% 
% 
% % Receives OptiTrack data.
% % ---------------------------------------------------------------------------------------------------------------------
% function [latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti] = ...
%     receiveAndProcessOpti(u_opti, t_exp, latest_opti, latest_opti_time, latest_opti_quality, last_opti_receive_time, print_counter_opti)
% 
%     packet = readLatestUdpPacket(u_opti);
% 
%     if strlength(packet) == 0
%         return;
%     end
% 
%     opti = parseOptiPacket(packet);
% 
%     if ~opti.valid
%         fprintf('[MATLAB] Invalid OptiTrack packet: %s\n', packet);
%         return;
%     end
% 
%     latest_opti = opti.position;
%     latest_opti_time = opti.timestamp;
%     latest_opti_quality = opti.quality;
%     last_opti_receive_time = toc(t_exp);
% 
%     print_counter_opti = print_counter_opti + 1;
%     if mod(print_counter_opti, 200) == 0
%         fprintf('[MATLAB] OptiTrack Packet -> X: %.2f, Y: %.2f, Z: %.2f | Quality: %.2f\n', ...
%             latest_opti(1), latest_opti(2), latest_opti(3), latest_opti_quality);
%     end
% end
% 
% 
% % Checks if a sensor has recently produced data.
% % ---------------------------------------------------------------------------------------------------------------------
% function is_fresh = isSensorFresh(last_receive_time, current_time, freshness_timeout)
%     is_fresh = isfinite(last_receive_time) && ((current_time - last_receive_time) <= freshness_timeout);
% end
% 
% 
% % Publishes a geometry_msgs/Point if the requested publisher exists and the position is valid.
% % ---------------------------------------------------------------------------------------------------------------------
% function publishPointIfAvailable(ros_publishers, publisher_name, position)
%     if any(isnan(position))
%         return;
%     end
% 
%     try
%         if isfield(ros_publishers, publisher_name)
%             pub = ros_publishers.(publisher_name);
%             msg = ros_publishers.([publisher_name, '_msg']);
% 
%             msg.X = position(1);
%             msg.Y = position(2);
%             msg.Z = position(3);
% 
%             send(pub, msg);
%         end
%     catch ME
%         fprintf('[ROS] Publish error on %s: %s\n', publisher_name, ME.message);
%     end
% end
% 
% 
% % Placeholder for future UWB-over-ROS/custom PCB input handled inside MATLAB.
% % ---------------------------------------------------------------------------------------------------------------------
% function startUwbRosPlaceholder()
%     disp('[MASTER] UWB source is ROS/custom PCB.');
%     disp('[MASTER] UWB-over-ROS reading is a placeholder in this MATLAB script for now.');
% end
% 
% 
% % Placeholder for future GPS RTK ROS input handled inside MATLAB.
% % ---------------------------------------------------------------------------------------------------------------------
% function startGpsRtkPlaceholder(topic_name, frame_name)
%     fprintf('[MASTER] GPS RTK selected. Placeholder topic: %s | frame: %s\n', topic_name, frame_name);
%     disp('[MASTER] GPS RTK ROS reading is a placeholder in this MATLAB script for now.');
% end
% 
% 
% % Closes a file ID safely.
% % ---------------------------------------------------------------------------------------------------------------------
% function closeFileSafe(fid)
%     try
%         if fid > 0
%             fclose(fid);
%         end
%     catch
%     end
% end
