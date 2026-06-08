% =========================================================================
% READLIMO
% Author: Renzo Eisma
% Assistance note:
%   ChatGPT Pro 5.5 Thinking Extended was used to clean up variable names,
%   comments, line spacing, and general code structure.
%   The original concept, code logic, and project structure were created by Renzo Eisma.
% Date: 06/2026
%
% Purpose:
%   Small ROS reader for the Limo robot IMU.
%   This class does NOT control the robot. It only reads sensor data and
%   converts it to the standard imu_sample and angles structs used by the
%   new MATLAB localization structure.
%
% Current Limo topic:
%   /L1/imu    sensor_msgs/Imu
%
% Standard imu_sample output:
%   imu_sample.accel_body  = [ax ay az]
%   imu_sample.gyro_body   = [gx gy gz]
%   imu_sample.orientation = [roll pitch yaw]
%   imu_sample.timestamp
%   imu_sample.valid
%   imu_sample.source      = 'limo'
%
% Standard angles output:
%   angles.roll
%   angles.pitch
%   angles.yaw
%   angles.timestamp
%   angles.valid
%   angles.source = 'limo'
%
% Design notes:
%   - Limo is treated as a ground robot. The full [roll pitch yaw] fields are
%     still output to keep the format compatible with Bebop/custom PCB later.
%   - The control script should stay separate. ControlLimo.m or the existing
%     RobotControlCodeLimuClass.m should only receive final_position and
%     final_angles later.
% =========================================================================

classdef ReadLimo < handle

    % =====================================================================
    % USER CONFIGURATION
    % =====================================================================
    properties
        % ROS configuration
        ROS_NAMESPACE = "/L1";
        IMU_TOPIC = "/L1/imu";
        IMU_MSG_TYPE = "sensor_msgs/Imu";

        % Automatically try to subscribe during construction.
        % If ROS is not initialized yet, the object remains valid and
        % IsConnected remains false. connect() can be called after ROS starts.
        AUTO_CONNECT = true;

        % A sample is considered stale when it is older than this value.
        DATA_FRESH_TIMEOUT = 0.5;     % [s]

        % Print a short debug line every N samples. Set to Inf to disable.
        DEBUG_PRINT_EVERY = Inf;
    end

    % =====================================================================
    % INTERNAL STATE
    % =====================================================================
    properties
        imu_sub = [];
        IsConnected = false;

        LatestImuSample = struct();
        LatestAngles = struct();

        HasSample = false;
        NumSamples = 0;
        LastReceiveWallTime = NaN;    % posix time [s]
        LastRosTimestamp = NaN;       % ROS header stamp [s], if available
        LastStatus = "not_started";
    end

    % =====================================================================
    % PUBLIC METHODS
    % =====================================================================
    methods

        % =================================================================
        % Constructor
        % Usage:
        %   reader = ReadLimo()
        %   reader = ReadLimo("/L1")
        %   reader = ReadLimo(config_struct)
        % =================================================================
        function obj = ReadLimo(config_or_namespace)
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestAngles = obj.makeEmptyAngles();

            if nargin >= 1
                if isstruct(config_or_namespace)
                    obj.configure(config_or_namespace);
                elseif ischar(config_or_namespace) || isstring(config_or_namespace)
                    obj.ROS_NAMESPACE = string(config_or_namespace);
                    obj.IMU_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "imu");
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

            % If namespace was changed but topic was not explicitly changed,
            % rebuild the default topic.
            if isfield(config, 'ROS_NAMESPACE') && ~isfield(config, 'IMU_TOPIC')
                obj.IMU_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "imu");
            end
        end

        % =================================================================
        % Connect to ROS IMU topic
        % =================================================================
        function success = connect(obj)
            success = false;

            if obj.IsConnected
                success = true;
                return;
            end

            try
                fprintf('[ReadLimo] Subscribing to %s (%s)...\n', char(obj.IMU_TOPIC), char(obj.IMU_MSG_TYPE));

                % Use the same simple rossubscriber style as the existing
                % Limo control class for compatibility.
                obj.imu_sub = rossubscriber(char(obj.IMU_TOPIC), char(obj.IMU_MSG_TYPE), @obj.imuCallback);

                obj.IsConnected = true;
                obj.LastStatus = "connected";
                success = true;

            catch ME
                obj.IsConnected = false;
                obj.LastStatus = "connect_failed";
                fprintf('[ReadLimo] Could not subscribe to Limo IMU: %s\n', ME.message);
                fprintf('[ReadLimo] Make sure rosinit has been called and the topic exists.\n');
            end
        end

        % =================================================================
        % Disconnect / clear subscriber
        % =================================================================
        function disconnect(obj)
            try
                obj.imu_sub = [];
            catch
            end
            obj.IsConnected = false;
            obj.LastStatus = "disconnected";
        end

        % =================================================================
        % Return latest standard imu_sample
        % =================================================================
        function imu_sample = getImuSample(obj)
            imu_sample = obj.LatestImuSample;

            if ~obj.HasSample || ~obj.isDataFresh()
                imu_sample.valid = false;
                if ~obj.HasSample
                    imu_sample.status = "no_sample_yet";
                else
                    imu_sample.status = "stale_sample";
                end
            end
        end

        % =================================================================
        % Return latest standard angles struct
        % =================================================================
        function angles = getAngles(obj)
            angles = obj.LatestAngles;

            if ~obj.HasSample || ~obj.isDataFresh()
                angles.valid = false;
                if ~obj.HasSample
                    angles.status = "no_sample_yet";
                else
                    angles.status = "stale_sample";
                end
            end
        end

        % =================================================================
        % Check whether the newest sample is fresh enough
        % =================================================================
        function fresh = isDataFresh(obj, timeout)
            if nargin < 2 || isempty(timeout)
                timeout = obj.DATA_FRESH_TIMEOUT;
            end

            if ~obj.HasSample || isnan(obj.LastReceiveWallTime)
                fresh = false;
                return;
            end

            age = obj.getWallTimeSeconds() - obj.LastReceiveWallTime;
            fresh = age <= timeout;
        end

        % =================================================================
        % Optional helper for master script
        % =================================================================
        function [imu_sample, angles] = getLatest(obj)
            imu_sample = obj.getImuSample();
            angles = obj.getAngles();
        end

        % =================================================================
        % Reset stored samples
        % =================================================================
        function reset(obj)
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestAngles = obj.makeEmptyAngles();
            obj.HasSample = false;
            obj.NumSamples = 0;
            obj.LastReceiveWallTime = NaN;
            obj.LastRosTimestamp = NaN;
            obj.LastStatus = "reset";
        end
    end

    % =====================================================================
    % ROS CALLBACKS
    % =====================================================================
    methods

        % =================================================================
        % ROS IMU callback
        % =================================================================
        function imuCallback(obj, ~, message)
            try
                receive_time = obj.getWallTimeSeconds();
                ros_stamp = obj.getRosTimestamp(message);

                accel = obj.extractVector3(message.LinearAcceleration);
                gyro = obj.extractVector3(message.AngularVelocity);

                q = [message.Orientation.W, ...
                     message.Orientation.X, ...
                     message.Orientation.Y, ...
                     message.Orientation.Z];

                [orientation, orientation_valid] = obj.quaternionToRollPitchYaw(q);

                imu_sample = obj.makeEmptyImuSample();
                imu_sample.accel_body = accel;
                imu_sample.gyro_body = gyro;
                imu_sample.orientation = orientation;
                imu_sample.orientation_valid = orientation_valid;
                imu_sample.timestamp = receive_time;
                imu_sample.ros_timestamp = ros_stamp;
                imu_sample.valid = true;
                imu_sample.source = "limo";
                imu_sample.status = "ok";

                angles = obj.makeEmptyAngles();
                angles.roll = orientation(1);
                angles.pitch = orientation(2);
                angles.yaw = orientation(3);
                angles.timestamp = receive_time;
                angles.ros_timestamp = ros_stamp;
                angles.valid = orientation_valid;
                angles.source = "limo";
                if orientation_valid
                    angles.status = "ok";
                else
                    angles.status = "orientation_not_available";
                end

                obj.LatestImuSample = imu_sample;
                obj.LatestAngles = angles;
                obj.HasSample = true;
                obj.NumSamples = obj.NumSamples + 1;
                obj.LastReceiveWallTime = receive_time;
                obj.LastRosTimestamp = ros_stamp;
                obj.LastStatus = "sample_received";

                if isfinite(obj.DEBUG_PRINT_EVERY) && obj.DEBUG_PRINT_EVERY > 0
                    if mod(obj.NumSamples, obj.DEBUG_PRINT_EVERY) == 0
                        fprintf('[ReadLimo] accel=[%.2f %.2f %.2f], gyro=[%.2f %.2f %.2f], rpy=[%.2f %.2f %.2f]\n', ...
                            accel(1), accel(2), accel(3), gyro(1), gyro(2), gyro(3), ...
                            orientation(1), orientation(2), orientation(3));
                    end
                end

            catch ME
                obj.LastStatus = "callback_error";
                fprintf('[ReadLimo] IMU callback error: %s\n', ME.message);
            end
        end
    end

    % =====================================================================
    % PRIVATE HELPER FUNCTIONS
    % =====================================================================
    methods (Access = private)

        % =================================================================
        % Standard empty IMU sample
        % =================================================================
        function imu_sample = makeEmptyImuSample(obj) %#ok<MANU>
            imu_sample = struct();
            imu_sample.accel_body = [NaN, NaN, NaN];
            imu_sample.gyro_body = [NaN, NaN, NaN];
            imu_sample.orientation = [NaN, NaN, NaN];
            imu_sample.orientation_valid = false;
            imu_sample.timestamp = NaN;
            imu_sample.ros_timestamp = NaN;
            imu_sample.valid = false;
            imu_sample.source = "limo";
            imu_sample.status = "empty";
        end

        % =================================================================
        % Standard empty angle sample
        % =================================================================
        function angles = makeEmptyAngles(obj) %#ok<MANU>
            angles = struct();
            angles.roll = NaN;
            angles.pitch = NaN;
            angles.yaw = NaN;
            angles.timestamp = NaN;
            angles.ros_timestamp = NaN;
            angles.valid = false;
            angles.source = "limo";
            angles.status = "empty";
        end

        % =================================================================
        % Build topic from namespace and short topic name
        % =================================================================
        function topic = buildTopic(obj, namespace, topic_name) %#ok<INUSL>
            namespace = string(namespace);
            topic_name = string(topic_name);

            if strlength(namespace) == 0
                namespace = "/";
            end

            if ~startsWith(namespace, "/")
                namespace = "/" + namespace;
            end

            if endsWith(namespace, "/")
                topic = namespace + topic_name;
            else
                topic = namespace + "/" + topic_name;
            end
        end

        % =================================================================
        % Extract [x y z] from a geometry_msgs/Vector3-like object
        % =================================================================
        function v = extractVector3(obj, vector_msg) %#ok<INUSL>
            v = [double(vector_msg.X), double(vector_msg.Y), double(vector_msg.Z)];
        end

        % =================================================================
        % Get ROS timestamp from message header if available
        % =================================================================
        function t = getRosTimestamp(obj, message) %#ok<INUSL>
            t = NaN;
            try
                sec = double(message.Header.Stamp.Sec);
                nsec = double(message.Header.Stamp.Nsec);
                if sec > 0 || nsec > 0
                    t = sec + nsec * 1e-9;
                end
            catch
                t = NaN;
            end
        end

        % =================================================================
        % Current wall clock time as POSIX seconds
        % =================================================================
        function t = getWallTimeSeconds(obj) %#ok<MANU>
            t = posixtime(datetime('now', 'TimeZone', 'local'));
        end

        % =================================================================
        % Quaternion to roll-pitch-yaw
        % Input q = [w x y z]
        % Output orientation = [roll pitch yaw] in degrees
        % =================================================================
        function [orientation, valid] = quaternionToRollPitchYaw(obj, q) %#ok<INUSL>
            orientation = [NaN, NaN, NaN];
            valid = false;

            q = double(q(:).');
            if numel(q) ~= 4 || any(isnan(q)) || norm(q) < 1e-9
                return;
            end

            q = q ./ norm(q);
            qw = q(1);
            qx = q(2);
            qy = q(3);
            qz = q(4);

            % Roll, pitch, yaw using aerospace ZYX convention.
            roll = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));

            pitch_arg = 2*(qw*qy - qz*qx);
            pitch_arg = max(min(pitch_arg, 1), -1);  % numerical protection
            pitch = asin(pitch_arg);

            yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

            orientation = rad2deg([roll, pitch, yaw]);
            valid = true;
        end
    end
end
