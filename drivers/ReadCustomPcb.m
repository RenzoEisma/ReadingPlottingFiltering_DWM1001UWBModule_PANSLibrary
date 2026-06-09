% =========================================================================
% READCUSTOMPCB
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
%   Placeholder ROS reader for the custom UWB PCB data path.
%   This class keeps custom PCB details out of the main MATLAB master script
%   and always outputs the same standard formats used by the rest of the
%   localization framework.
%
% Custom PCB data path:
%   - ESP32-C6 sends data to ROS / micro-ROS.
%   - DWM1001C provides UWB position or raw ranges.
%   - BMI270 provides accelerometer and gyroscope data.
%   - Any raw-distance filtering or internal UWB positioning logic should be
%     handled inside this reader, not in MatlabMasterUWBControl.m.
%
% Standard outputs:
%   uwb_sample.position     = [x y z]
%   uwb_sample.quality
%   uwb_sample.timestamp
%   uwb_sample.valid
%   uwb_sample.source       = 'custom_pcb_ros'
%
%   imu_sample.accel_body   = [ax ay az]
%   imu_sample.gyro_body    = [gx gy gz]
%   imu_sample.orientation  = [roll pitch yaw]
%   imu_sample.timestamp
%   imu_sample.valid
%   imu_sample.source       = 'custom_pcb'
%
%   angles.roll / pitch / yaw
%
% Design notes:
%   - This file is intentionally simple and mostly future-proofing.
%   - It can be added to GitHub now without needing the PCB to exist yet.
%   - AUTO_CONNECT is false by default so it will not break tests when the
%     custom PCB ROS topics are not available.
% =========================================================================

classdef ReadCustomPcb < handle

    % =====================================================================
    % USER CONFIGURATION
    % =====================================================================
    properties
        % ROS namespace and topic names. These are placeholders and can be
        % changed later to match the ESP32-C6 / micro-ROS implementation.
        ROS_NAMESPACE = "/custom_pcb";

        UWB_POSITION_TOPIC = "/custom_pcb/uwb_position";
        IMU_TOPIC          = "/custom_pcb/imu";

        % Expected default ROS message types.
        % Supported for UWB in this placeholder:
        %   geometry_msgs/PoseStamped
        %   geometry_msgs/PointStamped
        %   geometry_msgs/Pose
        %   geometry_msgs/Point
        %   nav_msgs/Odometry
        UWB_MSG_TYPE = "geometry_msgs/PoseStamped";
        IMU_MSG_TYPE = "sensor_msgs/Imu";

        % Optional logging. Keep it simple: one CSV for UWB and one for IMU.
        LOG_ENABLE = true;
        UWB_LOG_FILE = "";
        IMU_LOG_FILE = "";

        % Automatically try to subscribe during construction.
        % False by default because this is a future placeholder.
        AUTO_CONNECT = false;

        % Data freshness timeout.
        DATA_FRESH_TIMEOUT = 0.5;     % [s]

        % Print debug every N samples. Set to Inf to disable.
        DEBUG_PRINT_EVERY = Inf;
    end

    % =====================================================================
    % INTERNAL STATE
    % =====================================================================
    properties
        uwb_sub = [];
        imu_sub = [];

        IsConnected = false;
        HasUwbSample = false;
        HasImuSample = false;

        LatestUwbSample = struct();
        LatestImuSample = struct();
        LatestAngles = struct();

        NumUwbSamples = 0;
        NumImuSamples = 0;

        LastUwbReceiveWallTime = NaN;
        LastImuReceiveWallTime = NaN;
        LastStatus = "not_started";
    end

    % =====================================================================
    % PUBLIC METHODS
    % =====================================================================
    methods

        % =================================================================
        % Constructor
        % Usage:
        %   reader = ReadCustomPcb()
        %   reader = ReadCustomPcb("/my_pcb")
        %   reader = ReadCustomPcb(config_struct)
        % =================================================================
        function obj = ReadCustomPcb(config_or_namespace)
            obj.LatestUwbSample = obj.makeEmptyUwbSample();
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestAngles = obj.makeEmptyAngles();

            if nargin >= 1
                if isstruct(config_or_namespace)
                    obj.configure(config_or_namespace);
                elseif ischar(config_or_namespace) || isstring(config_or_namespace)
                    obj.ROS_NAMESPACE = string(config_or_namespace);
                    obj.UWB_POSITION_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "uwb_position");
                    obj.IMU_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "imu");
                end
            end

            obj.initLogs();

            if obj.AUTO_CONNECT
                obj.connect();
            end
        end

        % =================================================================
        % Configure object from struct
        % =================================================================
        function configure(obj, config)
            names = fieldnames(config);
            for i = 1:numel(names)
                name = names{i};
                if isprop(obj, name)
                    obj.(name) = config.(name);
                end
            end

            if isfield(config, 'ROS_NAMESPACE')
                if ~isfield(config, 'UWB_POSITION_TOPIC')
                    obj.UWB_POSITION_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "uwb_position");
                end
                if ~isfield(config, 'IMU_TOPIC')
                    obj.IMU_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "imu");
                end
            end

            obj.initLogs();
        end

        % =================================================================
        % Connect to custom PCB ROS topics
        % =================================================================
        function success = connect(obj)
            success = false;
            uwb_ok = false;
            imu_ok = false;

            try
                fprintf('[ReadCustomPcb] Subscribing to %s (%s)...\n', char(obj.UWB_POSITION_TOPIC), char(obj.UWB_MSG_TYPE));
                obj.uwb_sub = rossubscriber(char(obj.UWB_POSITION_TOPIC), char(obj.UWB_MSG_TYPE), @obj.uwbCallback);
                uwb_ok = true;
            catch ME
                fprintf('[ReadCustomPcb] Could not subscribe to UWB topic: %s\n', ME.message);
            end

            try
                fprintf('[ReadCustomPcb] Subscribing to %s (%s)...\n', char(obj.IMU_TOPIC), char(obj.IMU_MSG_TYPE));
                obj.imu_sub = rossubscriber(char(obj.IMU_TOPIC), char(obj.IMU_MSG_TYPE), @obj.imuCallback);
                imu_ok = true;
            catch ME
                fprintf('[ReadCustomPcb] Could not subscribe to IMU topic: %s\n', ME.message);
            end

            obj.IsConnected = uwb_ok || imu_ok;
            success = obj.IsConnected;

            if success
                obj.LastStatus = "connected";
            else
                obj.LastStatus = "connect_failed";
                fprintf('[ReadCustomPcb] This is expected if the future PCB ROS topics do not exist yet.\n');
            end
        end

        % =================================================================
        % Disconnect / clear subscribers
        % =================================================================
        function disconnect(obj)
            try
                obj.uwb_sub = [];
                obj.imu_sub = [];
            catch
            end
            obj.IsConnected = false;
            obj.LastStatus = "disconnected";
        end

        % =================================================================
        % Return latest standard UWB sample
        % =================================================================
        function uwb_sample = getUwbSample(obj)
            uwb_sample = obj.LatestUwbSample;

            if ~obj.HasUwbSample || ~obj.isUwbDataFresh()
                uwb_sample.valid = false;
                if ~obj.HasUwbSample
                    uwb_sample.status = "no_uwb_sample_yet";
                else
                    uwb_sample.status = "stale_uwb_sample";
                end
            end
        end

        % =================================================================
        % Return latest standard IMU sample
        % =================================================================
        function imu_sample = getImuSample(obj)
            imu_sample = obj.LatestImuSample;

            if ~obj.HasImuSample || ~obj.isImuDataFresh()
                imu_sample.valid = false;
                if ~obj.HasImuSample
                    imu_sample.status = "no_imu_sample_yet";
                else
                    imu_sample.status = "stale_imu_sample";
                end
            end
        end

        % =================================================================
        % Return latest angles struct
        % =================================================================
        function angles = getAngles(obj)
            angles = obj.LatestAngles;

            if ~obj.HasImuSample || ~obj.isImuDataFresh()
                angles.valid = false;
                if ~obj.HasImuSample
                    angles.status = "no_imu_sample_yet";
                else
                    angles.status = "stale_imu_sample";
                end
            end
        end

        % =================================================================
        % Check freshness
        % =================================================================
        function fresh = isUwbDataFresh(obj, timeout)
            if nargin < 2 || isempty(timeout)
                timeout = obj.DATA_FRESH_TIMEOUT;
            end
            fresh = obj.HasUwbSample && ~isnan(obj.LastUwbReceiveWallTime) && ...
                    (obj.getWallTimeSeconds() - obj.LastUwbReceiveWallTime <= timeout);
        end

        function fresh = isImuDataFresh(obj, timeout)
            if nargin < 2 || isempty(timeout)
                timeout = obj.DATA_FRESH_TIMEOUT;
            end
            fresh = obj.HasImuSample && ~isnan(obj.LastImuReceiveWallTime) && ...
                    (obj.getWallTimeSeconds() - obj.LastImuReceiveWallTime <= timeout);
        end

        % =================================================================
        % Optional helper for manual tests without ROS
        % =================================================================
        function setFakeUwbSample(obj, position, quality, timestamp)
            if nargin < 3 || isempty(quality)
                quality = NaN;
            end
            if nargin < 4 || isempty(timestamp)
                timestamp = obj.getWallTimeSeconds();
            end

            sample = obj.makeEmptyUwbSample();
            sample.position = double(position(:)).';
            sample.quality = quality;
            sample.timestamp = timestamp;
            sample.valid = true;
            sample.status = "fake_sample";

            obj.LatestUwbSample = sample;
            obj.HasUwbSample = true;
            obj.NumUwbSamples = obj.NumUwbSamples + 1;
            obj.LastUwbReceiveWallTime = obj.getWallTimeSeconds();
            obj.logUwbSample(sample);
        end

        % =================================================================
        % Placeholder for raw-distance processing
        % =================================================================
        function uwb_sample = processRawAnchorDistances(obj, raw_ranges)
            %#ok<INUSD>
            % FUTURE WORK:
            %   This is where raw distances from the DWM1001C/custom firmware
            %   should be processed if the PCB sends anchor ranges instead of
            %   already calculated XYZ.
            %
            % Possible processing steps:
            %   1. Read anchor IDs, ranges, signal quality and timestamps.
            %   2. Reject bad anchors/ranges using quality, jump checks or CIR.
            %   3. Calculate position from ranges using multilateration.
            %   4. Optionally run an internal EKF/tightly-coupled filter.
            %   5. Output the same standard uwb_sample as the PANS/Python path.
            %
            % Design note:
            %   MatlabMasterUWBControl.m should not need to know about any of
            %   this. It should only receive a normal uwb_sample.
            uwb_sample = obj.makeEmptyUwbSample();
            uwb_sample.status = "raw_range_processing_not_implemented";
        end
    end

    % =====================================================================
    % CALLBACKS
    % =====================================================================
    methods (Access = private)

        % =================================================================
        % UWB position callback
        % =================================================================
        function uwbCallback(obj, ~, msg)
            sample = obj.makeEmptyUwbSample();
            sample.timestamp = obj.extractTimestamp(msg);
            sample.position = obj.extractPosition(msg);
            sample.quality = obj.extractQuality(msg);
            sample.valid = all(isfinite(sample.position));

            if sample.valid
                sample.status = "ok";
            else
                sample.status = "invalid_position";
            end

            obj.LatestUwbSample = sample;
            obj.HasUwbSample = true;
            obj.NumUwbSamples = obj.NumUwbSamples + 1;
            obj.LastUwbReceiveWallTime = obj.getWallTimeSeconds();

            obj.logUwbSample(sample);

            if isfinite(obj.DEBUG_PRINT_EVERY) && obj.DEBUG_PRINT_EVERY > 0 && ...
                    mod(obj.NumUwbSamples, obj.DEBUG_PRINT_EVERY) == 0
                fprintf('[ReadCustomPcb] UWB: %.3f %.3f %.3f valid=%d\n', ...
                    sample.position(1), sample.position(2), sample.position(3), sample.valid);
            end
        end

        % =================================================================
        % IMU callback
        % =================================================================
        function imuCallback(obj, ~, msg)
            imu_sample = obj.makeEmptyImuSample();
            angles = obj.makeEmptyAngles();

            imu_sample.timestamp = obj.extractTimestamp(msg);
            imu_sample.accel_body = [double(msg.LinearAcceleration.X), ...
                                     double(msg.LinearAcceleration.Y), ...
                                     double(msg.LinearAcceleration.Z)];
            imu_sample.gyro_body = [double(msg.AngularVelocity.X), ...
                                    double(msg.AngularVelocity.Y), ...
                                    double(msg.AngularVelocity.Z)];

            q = [double(msg.Orientation.W), double(msg.Orientation.X), ...
                 double(msg.Orientation.Y), double(msg.Orientation.Z)];
            rpy = obj.quatToRpy(q);

            imu_sample.orientation = rpy;
            imu_sample.valid = all(isfinite(imu_sample.accel_body)) || all(isfinite(imu_sample.gyro_body)) || all(isfinite(rpy));
            imu_sample.status = "ok";

            angles.roll = rpy(1);
            angles.pitch = rpy(2);
            angles.yaw = rpy(3);
            angles.timestamp = imu_sample.timestamp;
            angles.valid = imu_sample.valid && all(isfinite(rpy));
            angles.status = "ok";

            obj.LatestImuSample = imu_sample;
            obj.LatestAngles = angles;
            obj.HasImuSample = true;
            obj.NumImuSamples = obj.NumImuSamples + 1;
            obj.LastImuReceiveWallTime = obj.getWallTimeSeconds();

            obj.logImuSample(imu_sample, angles);
        end
    end

    % =====================================================================
    % HELPER METHODS
    % =====================================================================
    methods (Access = private)

        function sample = makeEmptyUwbSample(obj) %#ok<MANU>
            sample = struct();
            sample.position = [NaN NaN NaN];
            sample.quality = NaN;
            sample.timestamp = NaN;
            sample.valid = false;
            sample.source = "custom_pcb_ros";
            sample.status = "empty";
        end

        function sample = makeEmptyImuSample(obj) %#ok<MANU>
            sample = struct();
            sample.accel_body = [NaN NaN NaN];
            sample.gyro_body = [NaN NaN NaN];
            sample.orientation = [NaN NaN NaN];
            sample.timestamp = NaN;
            sample.valid = false;
            sample.source = "custom_pcb";
            sample.status = "empty";
        end

        function angles = makeEmptyAngles(obj) %#ok<MANU>
            angles = struct();
            angles.roll = NaN;
            angles.pitch = NaN;
            angles.yaw = NaN;
            angles.timestamp = NaN;
            angles.valid = false;
            angles.source = "custom_pcb";
            angles.status = "empty";
        end

        function initLogs(obj)
            if ~obj.LOG_ENABLE
                return;
            end

            if strlength(string(obj.UWB_LOG_FILE)) > 0
                obj.ensureCsvHeader(char(obj.UWB_LOG_FILE), ...
                    'timestamp,x,y,z,quality,valid,source,status\n');
            end

            if strlength(string(obj.IMU_LOG_FILE)) > 0
                obj.ensureCsvHeader(char(obj.IMU_LOG_FILE), ...
                    'timestamp,ax,ay,az,gx,gy,gz,roll,pitch,yaw,valid,source,status\n');
            end
        end

        function logUwbSample(obj, sample)
            if ~obj.LOG_ENABLE || strlength(string(obj.UWB_LOG_FILE)) == 0
                return;
            end

            fid = fopen(char(obj.UWB_LOG_FILE), 'a');
            if fid < 0
                return;
            end
            fprintf(fid, '%.6f,%.6f,%.6f,%.6f,%.6f,%d,%s,%s\n', ...
                sample.timestamp, sample.position(1), sample.position(2), sample.position(3), ...
                sample.quality, sample.valid, char(sample.source), char(sample.status));
            fclose(fid);
        end

        function logImuSample(obj, imu_sample, angles)
            if ~obj.LOG_ENABLE || strlength(string(obj.IMU_LOG_FILE)) == 0
                return;
            end

            fid = fopen(char(obj.IMU_LOG_FILE), 'a');
            if fid < 0
                return;
            end
            fprintf(fid, '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%s,%s\n', ...
                imu_sample.timestamp, ...
                imu_sample.accel_body(1), imu_sample.accel_body(2), imu_sample.accel_body(3), ...
                imu_sample.gyro_body(1), imu_sample.gyro_body(2), imu_sample.gyro_body(3), ...
                angles.roll, angles.pitch, angles.yaw, ...
                imu_sample.valid, char(imu_sample.source), char(imu_sample.status));
            fclose(fid);
        end

        function ensureCsvHeader(obj, filename, header) %#ok<INUSL>
            if exist(filename, 'file') ~= 2
                fid = fopen(filename, 'w');
                if fid >= 0
                    fprintf(fid, '%s', header);
                    fclose(fid);
                end
            end
        end

        function position = extractPosition(obj, msg) %#ok<INUSL>
            position = [NaN NaN NaN];

            try
                % geometry_msgs/PoseStamped
                position = [double(msg.Pose.Position.X), double(msg.Pose.Position.Y), double(msg.Pose.Position.Z)];
                return;
            catch
            end

            try
                % geometry_msgs/PointStamped
                position = [double(msg.Point.X), double(msg.Point.Y), double(msg.Point.Z)];
                return;
            catch
            end

            try
                % nav_msgs/Odometry
                position = [double(msg.Pose.Pose.Position.X), double(msg.Pose.Pose.Position.Y), double(msg.Pose.Pose.Position.Z)];
                return;
            catch
            end

            try
                % geometry_msgs/Pose
                position = [double(msg.Position.X), double(msg.Position.Y), double(msg.Position.Z)];
                return;
            catch
            end

            try
                % geometry_msgs/Point
                position = [double(msg.X), double(msg.Y), double(msg.Z)];
                return;
            catch
            end
        end

        function quality = extractQuality(obj, msg) %#ok<INUSL>
            quality = NaN;

            % Placeholder. If the custom PCB later publishes quality or
            % covariance, map it here. For now this remains NaN.
            try
                if isfield(msg, 'Quality')
                    quality = double(msg.Quality);
                end
            catch
            end
        end

        function t = extractTimestamp(obj, msg)
            t = obj.getWallTimeSeconds();
            try
                sec = double(msg.Header.Stamp.Sec);
                nsec = double(msg.Header.Stamp.Nsec);
                t = sec + nsec * 1e-9;
            catch
            end
        end

        function topic = buildTopic(obj, namespace, suffix) %#ok<INUSL>
            namespace = string(namespace);
            suffix = string(suffix);
            if startsWith(suffix, "/")
                suffix = extractAfter(suffix, 1);
            end
            if namespace == "" || namespace == "/"
                topic = "/" + suffix;
            else
                if ~startsWith(namespace, "/")
                    namespace = "/" + namespace;
                end
                topic = namespace + "/" + suffix;
            end
        end

        function t = getWallTimeSeconds(obj) %#ok<MANU>
            t = posixtime(datetime('now', 'TimeZone', 'UTC'));
        end
    end

    % =====================================================================
    % STATIC HELPERS
    % =====================================================================
    methods (Static)
        function rpy = quatToRpy(q)
            % q = [w x y z]
            if numel(q) ~= 4 || any(~isfinite(q)) || norm(q) < eps
                rpy = [NaN NaN NaN];
                return;
            end

            q = q ./ norm(q);
            w = q(1); x = q(2); y = q(3); z = q(4);

            % Roll
            sinr_cosp = 2 * (w*x + y*z);
            cosr_cosp = 1 - 2 * (x*x + y*y);
            roll = atan2(sinr_cosp, cosr_cosp);

            % Pitch
            sinp = 2 * (w*y - z*x);
            if abs(sinp) >= 1
                pitch = sign(sinp) * pi/2;
            else
                pitch = asin(sinp);
            end

            % Yaw
            siny_cosp = 2 * (w*z + x*y);
            cosy_cosp = 1 - 2 * (y*y + z*z);
            yaw = atan2(siny_cosp, cosy_cosp);

            rpy = [roll pitch yaw];
        end
    end
end
