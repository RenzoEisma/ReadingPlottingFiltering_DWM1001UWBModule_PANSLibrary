% =========================================================================
% READCUSTOMPCB
% =========================================================================
% Author: Renzo Eisma
% Date: 06/2026
% Assistance note:
% The original concept, code logic, and project structure were created by 
% Renzo Eisma. But in this case almost the full program was written by 
% ChatGPT Pro 5.5 Thinking Extended.
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
%   - BMI270 rotation offset from UWB coordinate frame is compensated for 
%     in this script.
%
% Expected ROS message formats:
%
% /uwb_pcb/position
%   Message type: std_msgs/Float32MultiArray
%   Data format:
%       Data = [x, y, z, quality]
%   Units:
%       x, y, z  = position in metres
%       quality  = optional UWB quality value. Use NaN if unavailable.
%
% /uwb_pcb/imu
%   Message type: sensor_msgs/Imu
%   Used fields:
%       LinearAcceleration.X/Y/Z
%       AngularVelocity.X/Y/Z
%   Units:
%       acceleration = m/s^2
%       gyroscope    = rad/s
%
% /uwb_pcb/ranges
%   Message type: std_msgs/Float32MultiArray
%   Data format:
%       Data = [anchor_id_1, range_m_1, quality_1,
%               anchor_id_2, range_m_2, quality_2,
%               ...]
%   Units:
%       anchor_id = numeric anchor ID
%       range_m   = tag-to-anchor distance in metres
%       quality   = optional quality value. Use NaN if unavailable.
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
%   angles.roll / pitch / yaw (in degrees)
%
% Design notes:
%   - This file is intentionally simple and mostly future-proofing.
%   - AUTO_CONNECT is false by default so it will not break tests when the
%     custom PCB ROS topics are not available.
%
% Future improvements:
%   - Have accelerometer and gyroscope coordinate transformation be done
%   inside the ESP32 before sending to ROS
%   - Integrate custom trilateration
%   - Update MasterControlScript to support this script
% =========================================================================

classdef ReadCustomPcb < handle

    % =====================================================================
    % USER CONFIGURATION
    % =====================================================================
    properties
        % -------------------------------------------------------------
        % ROS topic configuration
        % -------------------------------------------------------------
        ROS_NAMESPACE = "/uwb_pcb";

        POSITION_TOPIC = "/uwb_pcb/position";
        IMU_TOPIC      = "/uwb_pcb/imu";
        RANGES_TOPIC   = "/uwb_pcb/ranges";

        POSITION_MSG_TYPE = "std_msgs/Float32MultiArray";
        IMU_MSG_TYPE      = "sensor_msgs/Imu";
        RANGES_MSG_TYPE   = "std_msgs/Float32MultiArray";

        % -------------------------------------------------------------
        % UWB position mode
        % -------------------------------------------------------------
        % "CALCULATED_POSITION"   = use /uwb_pcb/position directly
        % "CUSTOM_TRILATERATION"  = use /uwb_pcb/ranges, placeholder for now
        POSITION_MODE = "CALCULATED_POSITION";

        % -------------------------------------------------------------
        % IMU axis alignment
        % -------------------------------------------------------------
        % Fixed rotation from raw IMU frame to PCB/body frame.
        % Change these values when the BMI270 axis orientation is known.
        %
        % Definition:
        %   accel_body = R_imu_to_body * accel_raw
        %   gyro_body  = R_imu_to_body * gyro_raw
        %
        % Angles are in degrees and use Rz(yaw) * Ry(pitch) * Rx(roll).
        IMU_TO_BODY_ROTATION_DEG = [0, 0, 0];  % [roll pitch yaw] [deg]

        % -------------------------------------------------------------
        % Runtime behaviour
        % -------------------------------------------------------------
        AUTO_CONNECT = false;
        DATA_FRESH_TIMEOUT = 0.5;     % [s]
        DEBUG_PRINT_EVERY = Inf;      % Set to a number to enable debug prints
    end

    % =====================================================================
    % INTERNAL STATE
    % =====================================================================
    properties
        position_sub = [];
        imu_sub = [];
        ranges_sub = [];

        IsConnected = false;

        HasPositionSample = false;
        HasImuSample = false;
        HasRangesSample = false;

        LatestUwbSample = struct();
        LatestImuSample = struct();
        LatestAngles = struct();
        LatestRangesSample = struct();

        NumPositionSamples = 0;
        NumImuSamples = 0;
        NumRangesSamples = 0;

        LastPositionReceiveWallTime = NaN;
        LastImuReceiveWallTime = NaN;
        LastRangesReceiveWallTime = NaN;

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
        %   reader = ReadCustomPcb("/uwb_pcb")
        %   reader = ReadCustomPcb(config_struct)
        % =================================================================
        function obj = ReadCustomPcb(config_or_namespace)
            obj.LatestUwbSample = obj.makeEmptyUwbSample();
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestAngles = obj.makeEmptyAngles();
            obj.LatestRangesSample = obj.makeEmptyRangesSample();

            if nargin >= 1
                if isstruct(config_or_namespace)
                    obj.configure(config_or_namespace);
                elseif ischar(config_or_namespace) || isstring(config_or_namespace)
                    obj.ROS_NAMESPACE = string(config_or_namespace);
                    obj.POSITION_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "position");
                    obj.IMU_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "imu");
                    obj.RANGES_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "ranges");
                end
            end

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
                if ~isfield(config, 'POSITION_TOPIC')
                    obj.POSITION_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "position");
                end
                if ~isfield(config, 'IMU_TOPIC')
                    obj.IMU_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "imu");
                end
                if ~isfield(config, 'RANGES_TOPIC')
                    obj.RANGES_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "ranges");
                end
            end
        end

        % =================================================================
        % Connect to ROS topics
        % =================================================================
        function success = connect(obj)
            success = false;
            position_ok = false;
            imu_ok = false;
            ranges_ok = false;

            try
                fprintf('[ReadCustomPcb] Subscribing to %s (%s)...\n', ...
                    char(obj.POSITION_TOPIC), char(obj.POSITION_MSG_TYPE));

                obj.position_sub = rossubscriber( ...
                    char(obj.POSITION_TOPIC), ...
                    char(obj.POSITION_MSG_TYPE), ...
                    @obj.positionCallback);

                position_ok = true;

            catch ME
                fprintf('[ReadCustomPcb] Could not subscribe to position topic: %s\n', ME.message);
            end

            try
                fprintf('[ReadCustomPcb] Subscribing to %s (%s)...\n', ...
                    char(obj.IMU_TOPIC), char(obj.IMU_MSG_TYPE));

                obj.imu_sub = rossubscriber( ...
                    char(obj.IMU_TOPIC), ...
                    char(obj.IMU_MSG_TYPE), ...
                    @obj.imuCallback);

                imu_ok = true;

            catch ME
                fprintf('[ReadCustomPcb] Could not subscribe to IMU topic: %s\n', ME.message);
            end

            try
                fprintf('[ReadCustomPcb] Subscribing to %s (%s)...\n', ...
                    char(obj.RANGES_TOPIC), char(obj.RANGES_MSG_TYPE));

                obj.ranges_sub = rossubscriber( ...
                    char(obj.RANGES_TOPIC), ...
                    char(obj.RANGES_MSG_TYPE), ...
                    @obj.rangesCallback);

                ranges_ok = true;

            catch ME
                fprintf('[ReadCustomPcb] Could not subscribe to ranges topic: %s\n', ME.message);
            end

            obj.IsConnected = position_ok || imu_ok || ranges_ok;
            success = obj.IsConnected;

            if success
                obj.LastStatus = "connected";
            else
                obj.LastStatus = "connect_failed";
            end
        end

        % =================================================================
        % Disconnect / clear subscribers
        % =================================================================
        function disconnect(obj)
            try
                obj.position_sub = [];
                obj.imu_sub = [];
                obj.ranges_sub = [];
            catch
            end

            obj.IsConnected = false;
            obj.LastStatus = "disconnected";
        end

        % =================================================================
        % Get latest UWB position sample
        % =================================================================
        function uwb_sample = getUwbSample(obj)
            mode = upper(string(obj.POSITION_MODE));

            switch mode
                case "CALCULATED_POSITION"
                    uwb_sample = obj.LatestUwbSample;

                    if ~obj.HasPositionSample || ~obj.isPositionDataFresh()
                        uwb_sample.valid = false;

                        if ~obj.HasPositionSample
                            uwb_sample.status = "no_position_sample_yet";
                        else
                            uwb_sample.status = "stale_position_sample";
                        end
                    end

                case "CUSTOM_TRILATERATION"
                    uwb_sample = obj.processRangesPlaceholder();

                otherwise
                    uwb_sample = obj.makeEmptyUwbSample();
                    uwb_sample.status = "invalid_position_mode";
            end
        end

        % Alternative name if a future master script expects getPosition().
        function uwb_sample = getPosition(obj)
            uwb_sample = obj.getUwbSample();
        end

        % =================================================================
        % Get latest IMU sample
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
        % Get latest angles struct
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
        % Limo/Bebop-reader style helper
        % =================================================================
        function [imu_sample, angles] = getLatest(obj)
            imu_sample = obj.getImuSample();
            angles = obj.getAngles();
        end

        % =================================================================
        % Get latest raw ranges
        % =================================================================
        function ranges_sample = getRangesSample(obj)
            ranges_sample = obj.LatestRangesSample;

            if ~obj.HasRangesSample || ~obj.isRangesDataFresh()
                ranges_sample.valid = false;

                if ~obj.HasRangesSample
                    ranges_sample.status = "no_ranges_sample_yet";
                else
                    ranges_sample.status = "stale_ranges_sample";
                end
            end
        end

        % =================================================================
        % Freshness checks
        % =================================================================
        function fresh = isPositionDataFresh(obj, timeout)
            if nargin < 2 || isempty(timeout)
                timeout = obj.DATA_FRESH_TIMEOUT;
            end

            fresh = obj.HasPositionSample && ...
                    isfinite(obj.LastPositionReceiveWallTime) && ...
                    (obj.getWallTimeSeconds() - obj.LastPositionReceiveWallTime <= timeout);
        end

        function fresh = isImuDataFresh(obj, timeout)
            if nargin < 2 || isempty(timeout)
                timeout = obj.DATA_FRESH_TIMEOUT;
            end

            fresh = obj.HasImuSample && ...
                    isfinite(obj.LastImuReceiveWallTime) && ...
                    (obj.getWallTimeSeconds() - obj.LastImuReceiveWallTime <= timeout);
        end

        function fresh = isRangesDataFresh(obj, timeout)
            if nargin < 2 || isempty(timeout)
                timeout = obj.DATA_FRESH_TIMEOUT;
            end

            fresh = obj.HasRangesSample && ...
                    isfinite(obj.LastRangesReceiveWallTime) && ...
                    (obj.getWallTimeSeconds() - obj.LastRangesReceiveWallTime <= timeout);
        end

        % =================================================================
        % Reset stored samples
        % =================================================================
        function reset(obj)
            obj.LatestUwbSample = obj.makeEmptyUwbSample();
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestAngles = obj.makeEmptyAngles();
            obj.LatestRangesSample = obj.makeEmptyRangesSample();

            obj.HasPositionSample = false;
            obj.HasImuSample = false;
            obj.HasRangesSample = false;

            obj.NumPositionSamples = 0;
            obj.NumImuSamples = 0;
            obj.NumRangesSamples = 0;

            obj.LastPositionReceiveWallTime = NaN;
            obj.LastImuReceiveWallTime = NaN;
            obj.LastRangesReceiveWallTime = NaN;

            obj.LastStatus = "reset";
        end
    end

    % =====================================================================
    % ROS CALLBACKS
    % =====================================================================
    methods

        % =================================================================
        % Position callback
        % Expected std_msgs/Float32MultiArray:
        %   Data = [x, y, z, quality]
        % quality is optional. If missing, quality = NaN.
        % =================================================================
        function positionCallback(obj, ~, msg)
            try
                receive_time = obj.getWallTimeSeconds();
                data = double(msg.Data(:)).';

                sample = obj.makeEmptyUwbSample();
                sample.timestamp = receive_time;
                sample.source = "custom_pcb_ros";

                if numel(data) >= 3
                    sample.position = data(1:3);
                    sample.valid = all(isfinite(sample.position));

                    if numel(data) >= 4
                        sample.quality = data(4);
                    else
                        sample.quality = NaN;
                    end
                else
                    sample.valid = false;
                end

                if sample.valid
                    sample.status = "ok";
                else
                    sample.status = "invalid_position_message";
                end

                obj.LatestUwbSample = sample;
                obj.HasPositionSample = true;
                obj.NumPositionSamples = obj.NumPositionSamples + 1;
                obj.LastPositionReceiveWallTime = receive_time;
                obj.LastStatus = "position_received";

                if isfinite(obj.DEBUG_PRINT_EVERY) && obj.DEBUG_PRINT_EVERY > 0
                    if mod(obj.NumPositionSamples, obj.DEBUG_PRINT_EVERY) == 0
                        fprintf('[ReadCustomPcb] position=[%.3f %.3f %.3f], valid=%d\n', ...
                            sample.position(1), sample.position(2), sample.position(3), sample.valid);
                    end
                end

            catch ME
                obj.LastStatus = "position_callback_error";
                fprintf('[ReadCustomPcb] Position callback error: %s\n', ME.message);
            end
        end

        % =================================================================
        % IMU callback
        % Expected sensor_msgs/Imu:
        %   LinearAcceleration = accelerometer data
        %   AngularVelocity    = gyroscope data
        %
        % The raw IMU vectors are rotated into the PCB/body frame using
        % IMU_TO_BODY_ROTATION_DEG.
        % =================================================================
        function imuCallback(obj, ~, msg)
            try
                receive_time = obj.getWallTimeSeconds();

                accel_raw = [
                    double(msg.LinearAcceleration.X);
                    double(msg.LinearAcceleration.Y);
                    double(msg.LinearAcceleration.Z)
                ];

                gyro_raw = [
                    double(msg.AngularVelocity.X);
                    double(msg.AngularVelocity.Y);
                    double(msg.AngularVelocity.Z)
                ];

                R_imu_to_body = obj.rotationMatrixFromRpyDeg(obj.IMU_TO_BODY_ROTATION_DEG);

                accel_body = R_imu_to_body * accel_raw;
                gyro_body = R_imu_to_body * gyro_raw;

                imu_sample = obj.makeEmptyImuSample();
                imu_sample.accel_body = accel_body(:).';
                imu_sample.gyro_body = gyro_body(:).';
                imu_sample.orientation = [NaN NaN NaN];
                imu_sample.timestamp = receive_time;
                imu_sample.valid = all(isfinite(imu_sample.accel_body)) || ...
                                   all(isfinite(imu_sample.gyro_body));
                imu_sample.source = "custom_pcb";

                if imu_sample.valid
                    imu_sample.status = "ok";
                else
                    imu_sample.status = "invalid_imu_message";
                end

                % The custom PCB currently only provides accelerometer and
                % gyroscope data. No roll/pitch/yaw estimate is produced here.
                angles = obj.makeEmptyAngles();
                angles.timestamp = receive_time;
                angles.valid = false;
                angles.status = "orientation_not_available";

                obj.LatestImuSample = imu_sample;
                obj.LatestAngles = angles;
                obj.HasImuSample = true;
                obj.NumImuSamples = obj.NumImuSamples + 1;
                obj.LastImuReceiveWallTime = receive_time;
                obj.LastStatus = "imu_received";

                if isfinite(obj.DEBUG_PRINT_EVERY) && obj.DEBUG_PRINT_EVERY > 0
                    if mod(obj.NumImuSamples, obj.DEBUG_PRINT_EVERY) == 0
                        fprintf('[ReadCustomPcb] accel=[%.3f %.3f %.3f], gyro=[%.3f %.3f %.3f], valid=%d\n', ...
                            imu_sample.accel_body(1), imu_sample.accel_body(2), imu_sample.accel_body(3), ...
                            imu_sample.gyro_body(1), imu_sample.gyro_body(2), imu_sample.gyro_body(3), ...
                            imu_sample.valid);
                    end
                end

            catch ME
                obj.LastStatus = "imu_callback_error";
                fprintf('[ReadCustomPcb] IMU callback error: %s\n', ME.message);
            end
        end

        % =================================================================
        % Ranges callback
        % Expected std_msgs/Float32MultiArray:
        %   Data = [anchor_id_1, range_m_1, quality_1,
        %           anchor_id_2, range_m_2, quality_2,
        %           ...]
        %
        % This is stored for future custom trilateration.
        % =================================================================
        function rangesCallback(obj, ~, msg)
            try
                receive_time = obj.getWallTimeSeconds();
                data = double(msg.Data(:)).';

                ranges_sample = obj.makeEmptyRangesSample();
                ranges_sample.timestamp = receive_time;
                ranges_sample.source = "custom_pcb_ranges";

                if isempty(data) || mod(numel(data), 3) ~= 0
                    ranges_sample.valid = false;
                    ranges_sample.status = "invalid_ranges_message";
                else
                    reshaped = reshape(data, 3, []).';

                    ranges_sample.anchor_ids = reshaped(:, 1).';
                    ranges_sample.ranges_m = reshaped(:, 2).';
                    ranges_sample.quality = reshaped(:, 3).';
                    ranges_sample.valid = all(isfinite(ranges_sample.anchor_ids)) && ...
                                          all(isfinite(ranges_sample.ranges_m));
                    ranges_sample.status = "ok";
                end

                obj.LatestRangesSample = ranges_sample;
                obj.HasRangesSample = true;
                obj.NumRangesSamples = obj.NumRangesSamples + 1;
                obj.LastRangesReceiveWallTime = receive_time;
                obj.LastStatus = "ranges_received";

                if isfinite(obj.DEBUG_PRINT_EVERY) && obj.DEBUG_PRINT_EVERY > 0
                    if mod(obj.NumRangesSamples, obj.DEBUG_PRINT_EVERY) == 0
                        fprintf('[ReadCustomPcb] ranges count=%d, valid=%d\n', ...
                            numel(ranges_sample.ranges_m), ranges_sample.valid);
                    end
                end

            catch ME
                obj.LastStatus = "ranges_callback_error";
                fprintf('[ReadCustomPcb] Ranges callback error: %s\n', ME.message);
            end
        end
    end

    % =====================================================================
    % PRIVATE HELPER METHODS
    % =====================================================================
    methods (Access = private)

        % =================================================================
        % Placeholder custom trilateration
        % =================================================================
        function uwb_sample = processRangesPlaceholder(obj)
            ranges_sample = obj.getRangesSample();

            uwb_sample = obj.makeEmptyUwbSample();
            uwb_sample.timestamp = obj.getWallTimeSeconds();
            uwb_sample.source = "custom_pcb_trilateration";
            uwb_sample.valid = false;

            if ~ranges_sample.valid
                uwb_sample.status = "custom_trilateration_no_valid_ranges";
            else
                uwb_sample.status = "custom_trilateration_not_implemented";
            end
        end

        % =================================================================
        % Empty standard UWB sample
        % =================================================================
        function sample = makeEmptyUwbSample(obj) %#ok<MANU>
            sample = struct();
            sample.position = [NaN NaN NaN];
            sample.quality = NaN;
            sample.timestamp = NaN;
            sample.valid = false;
            sample.source = "custom_pcb_ros";
            sample.status = "empty";
        end

        % =================================================================
        % Empty standard IMU sample
        % =================================================================
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

        % =================================================================
        % Empty standard angles sample
        % =================================================================
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

        % =================================================================
        % Empty ranges sample
        % =================================================================
        function sample = makeEmptyRangesSample(obj) %#ok<MANU>
            sample = struct();
            sample.anchor_ids = [];
            sample.ranges_m = [];
            sample.quality = [];
            sample.timestamp = NaN;
            sample.valid = false;
            sample.source = "custom_pcb_ranges";
            sample.status = "empty";
        end

        % =================================================================
        % Build topic from namespace and suffix
        % =================================================================
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

        % =================================================================
        % Current wall-clock time as POSIX seconds
        % =================================================================
        function t = getWallTimeSeconds(obj) %#ok<MANU>
            t = posixtime(datetime('now', 'TimeZone', 'UTC'));
        end

        % =================================================================
        % Rotation matrix from roll/pitch/yaw in degrees
        % Rotation order: Rz(yaw) * Ry(pitch) * Rx(roll)
        % =================================================================
        function R = rotationMatrixFromRpyDeg(obj, rpy_deg) %#ok<INUSL>
            roll = rpy_deg(1);
            pitch = rpy_deg(2);
            yaw = rpy_deg(3);

            cr = cosd(roll);  sr = sind(roll);
            cp = cosd(pitch); sp = sind(pitch);
            cy = cosd(yaw);   sy = sind(yaw);

            Rx = [1 0 0; ...
                  0 cr -sr; ...
                  0 sr  cr];

            Ry = [ cp 0 sp; ...
                   0  1 0; ...
                  -sp 0 cp];

            Rz = [cy -sy 0; ...
                  sy  cy 0; ...
                  0   0  1];

            R = Rz * Ry * Rx;
        end
    end
end