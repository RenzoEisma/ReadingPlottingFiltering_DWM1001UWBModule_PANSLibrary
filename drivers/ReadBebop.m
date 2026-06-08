% =========================================================================
% READBEBOP
% Author: Renzo Eisma
% Assistance note:
%   ChatGPT Pro 5.5 Thinking Extended was used to clean up variable names,
%   comments, line spacing, and general code structure.
%   The original concept, code logic, and project structure were created by Renzo Eisma.
% Date: 06/2026
%
% Purpose:
%   Small ROS reader for the Bebop drone sensors.
%   This class does NOT control the drone. It only reads sensor/odometry data
%   and converts it to the standard imu_sample and angles structs used by the
%   new MATLAB localization structure.
%
% Current/default Bebop topics:
%   /B1/imu     sensor_msgs/Imu       if available from the Bebop driver
%   /B1/odom    nav_msgs/Odometry     fallback/source for orientation + gyro
%
% Standard imu_sample output:
%   imu_sample.accel_body  = [ax ay az]
%   imu_sample.gyro_body   = [gx gy gz]
%   imu_sample.orientation = [roll pitch yaw]
%   imu_sample.timestamp
%   imu_sample.valid
%   imu_sample.source      = 'bebop'
%
% Standard angles output:
%   angles.roll
%   angles.pitch
%   angles.yaw
%   angles.timestamp
%   angles.valid
%   angles.source = 'bebop'
%
% Design notes:
%   - The Bebop driver may not expose a /imu topic in every setup.
%   - If /imu is unavailable, /odom is still useful for orientation/yaw and
%     angular velocity, but acceleration will stay NaN.
%   - The control script should stay separate. ControlBebop.m or the existing
%     RobotControlCodeBebopClass.m should only receive final_position and
%     final_angles later.
% =========================================================================

classdef ReadBebop < handle

    % =====================================================================
    % USER CONFIGURATION
    % =====================================================================
    properties
        % ROS configuration
        ROS_NAMESPACE = "/B1";
        IMU_TOPIC = "/B1/imu";
        ODOM_TOPIC = "/B1/odom";

        IMU_MSG_TYPE = "sensor_msgs/Imu";
        ODOM_MSG_TYPE = "nav_msgs/Odometry";

        % Automatically try to subscribe during construction.
        % If ROS is not initialized yet, the object remains valid and
        % IsConnected remains false. connect() can be called after ROS starts.
        AUTO_CONNECT = true;

        % A sample is considered stale when it is older than this value.
        DATA_FRESH_TIMEOUT = 0.5;     % [s]

        % Prefer odometry orientation when both IMU and odom are available.
        % This is often useful because /odom is the state used by the Bebop
        % driver/control code.
        PREFER_ODOM_ORIENTATION = true;

        % Print a short debug line every N combined samples. Set to Inf to disable.
        DEBUG_PRINT_EVERY = Inf;
    end

    % =====================================================================
    % INTERNAL STATE
    % =====================================================================
    properties
        imu_sub = [];
        odom_sub = [];
        IsConnected = false;

        LatestImuRaw = struct();
        LatestOdomRaw = struct();

        LatestImuSample = struct();
        LatestAngles = struct();

        HasImuSample = false;
        HasOdomSample = false;
        NumImuSamples = 0;
        NumOdomSamples = 0;
        NumCombinedReads = 0;

        LastImuReceiveWallTime = NaN;     % posix time [s]
        LastOdomReceiveWallTime = NaN;    % posix time [s]
        LastRosTimestamp = NaN;           % ROS header stamp [s], if available
        LastStatus = "not_started";
    end

    % =====================================================================
    % PUBLIC METHODS
    % =====================================================================
    methods

        % =================================================================
        % Constructor
        % Usage:
        %   reader = ReadBebop()
        %   reader = ReadBebop("/B7")
        %   reader = ReadBebop(config_struct)
        % =================================================================
        function obj = ReadBebop(config_or_namespace)
            obj.LatestImuRaw = obj.makeEmptyImuSample();
            obj.LatestOdomRaw = obj.makeEmptyImuSample();
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestAngles = obj.makeEmptyAngles();

            if nargin >= 1
                if isstruct(config_or_namespace)
                    obj.configure(config_or_namespace);
                elseif ischar(config_or_namespace) || isstring(config_or_namespace)
                    obj.ROS_NAMESPACE = string(config_or_namespace);
                    obj.IMU_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "imu");
                    obj.ODOM_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "odom");
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

            % If namespace was changed but topics were not explicitly changed,
            % rebuild the default topics.
            if isfield(config, 'ROS_NAMESPACE')
                if ~isfield(config, 'IMU_TOPIC')
                    obj.IMU_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "imu");
                end
                if ~isfield(config, 'ODOM_TOPIC')
                    obj.ODOM_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "odom");
                end
            end
        end

        % =================================================================
        % Connect to ROS topics
        % =================================================================
        function success = connect(obj)
            success = false;

            if obj.IsConnected
                success = true;
                return;
            end

            imu_ok = false;
            odom_ok = false;

            % Subscribe to IMU topic if available. Some Bebop setups may not
            % actually publish it, but subscribing does not change control logic.
            try
                fprintf('[ReadBebop] Subscribing to %s (%s)...\n', char(obj.IMU_TOPIC), char(obj.IMU_MSG_TYPE));
                obj.imu_sub = rossubscriber(char(obj.IMU_TOPIC), char(obj.IMU_MSG_TYPE), @obj.imuCallback);
                imu_ok = true;
            catch ME
                fprintf('[ReadBebop] Could not subscribe to Bebop IMU topic: %s\n', ME.message);
            end

            % Subscribe to odom as fallback for orientation and angular velocity.
            try
                fprintf('[ReadBebop] Subscribing to %s (%s)...\n', char(obj.ODOM_TOPIC), char(obj.ODOM_MSG_TYPE));
                obj.odom_sub = rossubscriber(char(obj.ODOM_TOPIC), char(obj.ODOM_MSG_TYPE), @obj.odomCallback);
                odom_ok = true;
            catch ME
                fprintf('[ReadBebop] Could not subscribe to Bebop odom topic: %s\n', ME.message);
            end

            obj.IsConnected = imu_ok || odom_ok;
            success = obj.IsConnected;

            if success
                obj.LastStatus = "connected";
            else
                obj.LastStatus = "connect_failed";
                fprintf('[ReadBebop] Make sure rosinit has been called and the Bebop topics exist.\n');
            end
        end

        % =================================================================
        % Disconnect / clear subscribers
        % =================================================================
        function disconnect(obj)
            try
                obj.imu_sub = [];
                obj.odom_sub = [];
            catch
            end
            obj.IsConnected = false;
            obj.LastStatus = "disconnected";
        end

        % =================================================================
        % Return latest standard imu_sample
        % =================================================================
        function imu_sample = getImuSample(obj)
            [imu_sample, angles] = obj.buildCombinedOutput();
            obj.LatestImuSample = imu_sample;
            obj.LatestAngles = angles;
        end

        % =================================================================
        % Return latest standard angles struct
        % =================================================================
        function angles = getAngles(obj)
            [imu_sample, angles] = obj.buildCombinedOutput();
            obj.LatestImuSample = imu_sample;
            obj.LatestAngles = angles;
        end

        % =================================================================
        % Optional helper for master script
        % =================================================================
        function [imu_sample, angles] = getLatest(obj)
            [imu_sample, angles] = obj.buildCombinedOutput();
            obj.LatestImuSample = imu_sample;
            obj.LatestAngles = angles;
        end

        % =================================================================
        % Check whether newest data from either source is fresh enough
        % =================================================================
        function fresh = isDataFresh(obj, timeout)
            if nargin < 2 || isempty(timeout)
                timeout = obj.DATA_FRESH_TIMEOUT;
            end

            imu_fresh = obj.isImuFresh(timeout);
            odom_fresh = obj.isOdomFresh(timeout);
            fresh = imu_fresh || odom_fresh;
        end

        % =================================================================
        % Reset stored samples
        % =================================================================
        function reset(obj)
            obj.LatestImuRaw = obj.makeEmptyImuSample();
            obj.LatestOdomRaw = obj.makeEmptyImuSample();
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestAngles = obj.makeEmptyAngles();
            obj.HasImuSample = false;
            obj.HasOdomSample = false;
            obj.NumImuSamples = 0;
            obj.NumOdomSamples = 0;
            obj.NumCombinedReads = 0;
            obj.LastImuReceiveWallTime = NaN;
            obj.LastOdomReceiveWallTime = NaN;
            obj.LastRosTimestamp = NaN;
            obj.LastStatus = "reset";
        end
    end

    % =====================================================================
    % ROS CALLBACKS
    % =====================================================================
    methods

        % =================================================================
        % sensor_msgs/Imu callback
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
                imu_sample.accel_valid = all(isfinite(accel));
                imu_sample.gyro_valid = all(isfinite(gyro));
                imu_sample.orientation_valid = orientation_valid;
                imu_sample.timestamp = receive_time;
                imu_sample.ros_timestamp = ros_stamp;
                imu_sample.valid = true;
                imu_sample.source = "bebop_imu";
                imu_sample.status = "imu_sample_received";

                obj.LatestImuRaw = imu_sample;
                obj.HasImuSample = true;
                obj.NumImuSamples = obj.NumImuSamples + 1;
                obj.LastImuReceiveWallTime = receive_time;
                obj.LastRosTimestamp = ros_stamp;
                obj.LastStatus = "imu_sample_received";

            catch ME
                obj.LastStatus = "imu_callback_error";
                fprintf('[ReadBebop] IMU callback error: %s\n', ME.message);
            end
        end

        % =================================================================
        % nav_msgs/Odometry callback
        % =================================================================
        function odomCallback(obj, ~, message)
            try
                receive_time = obj.getWallTimeSeconds();
                ros_stamp = obj.getRosTimestamp(message);

                q = [message.Pose.Pose.Orientation.W, ...
                     message.Pose.Pose.Orientation.X, ...
                     message.Pose.Pose.Orientation.Y, ...
                     message.Pose.Pose.Orientation.Z];

                [orientation, orientation_valid] = obj.quaternionToRollPitchYaw(q);

                gyro = obj.extractVector3(message.Twist.Twist.Angular);
                linear_velocity = obj.extractVector3(message.Twist.Twist.Linear);

                odom_sample = obj.makeEmptyImuSample();
                odom_sample.accel_body = [NaN, NaN, NaN];
                odom_sample.gyro_body = gyro;
                odom_sample.orientation = orientation;
                odom_sample.linear_velocity_world = linear_velocity;
                odom_sample.accel_valid = false;
                odom_sample.gyro_valid = all(isfinite(gyro));
                odom_sample.orientation_valid = orientation_valid;
                odom_sample.timestamp = receive_time;
                odom_sample.ros_timestamp = ros_stamp;
                odom_sample.valid = orientation_valid || odom_sample.gyro_valid;
                odom_sample.source = "bebop_odom";
                odom_sample.status = "odom_sample_received";

                obj.LatestOdomRaw = odom_sample;
                obj.HasOdomSample = true;
                obj.NumOdomSamples = obj.NumOdomSamples + 1;
                obj.LastOdomReceiveWallTime = receive_time;
                obj.LastRosTimestamp = ros_stamp;
                obj.LastStatus = "odom_sample_received";

            catch ME
                obj.LastStatus = "odom_callback_error";
                fprintf('[ReadBebop] Odom callback error: %s\n', ME.message);
            end
        end
    end

    % =====================================================================
    % PRIVATE HELPER FUNCTIONS
    % =====================================================================
    methods (Access = private)

        % =================================================================
        % Combine /imu and /odom data into one standard output
        % =================================================================
        function [imu_sample, angles] = buildCombinedOutput(obj)
            imu_fresh = obj.isImuFresh(obj.DATA_FRESH_TIMEOUT);
            odom_fresh = obj.isOdomFresh(obj.DATA_FRESH_TIMEOUT);

            imu_sample = obj.makeEmptyImuSample();
            angles = obj.makeEmptyAngles();

            % Use real IMU acceleration/gyro when available.
            if imu_fresh
                imu_sample = obj.LatestImuRaw;
                imu_sample.source = "bebop";
            end

            % Use odom as fallback or preferred source for orientation.
            if odom_fresh
                odom_sample = obj.LatestOdomRaw;

                % If no IMU sample exists, start with odom sample.
                if ~imu_fresh
                    imu_sample = odom_sample;
                    imu_sample.source = "bebop";
                else
                    % Fill missing gyro from odom if needed.
                    if ~isfield(imu_sample, 'gyro_valid') || ~imu_sample.gyro_valid
                        imu_sample.gyro_body = odom_sample.gyro_body;
                        imu_sample.gyro_valid = odom_sample.gyro_valid;
                    end

                    % Prefer odom orientation when configured, or use it as fallback.
                    imu_orientation_missing = ~isfield(imu_sample, 'orientation_valid') || ~imu_sample.orientation_valid;
                    if obj.PREFER_ODOM_ORIENTATION || imu_orientation_missing
                        if odom_sample.orientation_valid
                            imu_sample.orientation = odom_sample.orientation;
                            imu_sample.orientation_valid = true;
                        end
                    end

                    % Keep linear velocity if odom provides it.
                    if isfield(odom_sample, 'linear_velocity_world')
                        imu_sample.linear_velocity_world = odom_sample.linear_velocity_world;
                    end
                end
            end

            % Standardize final fields.
            imu_sample.source = "bebop";
            imu_sample.valid = imu_fresh || odom_fresh;

            if ~imu_sample.valid
                imu_sample.status = "no_fresh_sensor_data";
            elseif imu_fresh && odom_fresh
                imu_sample.status = "imu_and_odom";
            elseif imu_fresh
                imu_sample.status = "imu_only";
            else
                imu_sample.status = "odom_only_no_accel";
            end

            % If only odom is available, acceleration is not available.
            if ~isfield(imu_sample, 'accel_valid')
                imu_sample.accel_valid = all(isfinite(imu_sample.accel_body));
            end
            if ~isfield(imu_sample, 'gyro_valid')
                imu_sample.gyro_valid = all(isfinite(imu_sample.gyro_body));
            end
            if ~isfield(imu_sample, 'orientation_valid')
                imu_sample.orientation_valid = all(isfinite(imu_sample.orientation));
            end

            % Timestamp: use newest available wall time.
            imu_sample.timestamp = max([obj.LastImuReceiveWallTime, obj.LastOdomReceiveWallTime], [], 'omitnan');
            if isempty(imu_sample.timestamp) || isnan(imu_sample.timestamp)
                imu_sample.timestamp = NaN;
            end
            imu_sample.ros_timestamp = obj.LastRosTimestamp;

            angles.roll = imu_sample.orientation(1);
            angles.pitch = imu_sample.orientation(2);
            angles.yaw = imu_sample.orientation(3);
            angles.timestamp = imu_sample.timestamp;
            angles.ros_timestamp = imu_sample.ros_timestamp;
            angles.valid = imu_sample.orientation_valid && imu_sample.valid;
            angles.source = "bebop";
            if angles.valid
                angles.status = "ok";
            elseif imu_sample.valid
                angles.status = "orientation_not_available";
            else
                angles.status = "no_fresh_sensor_data";
            end

            obj.NumCombinedReads = obj.NumCombinedReads + 1;
            if isfinite(obj.DEBUG_PRINT_EVERY) && obj.DEBUG_PRINT_EVERY > 0
                if mod(obj.NumCombinedReads, obj.DEBUG_PRINT_EVERY) == 0
                    fprintf('[ReadBebop] status=%s, accel=[%.2f %.2f %.2f], gyro=[%.2f %.2f %.2f], rpy=[%.2f %.2f %.2f]\n', ...
                        char(imu_sample.status), ...
                        imu_sample.accel_body(1), imu_sample.accel_body(2), imu_sample.accel_body(3), ...
                        imu_sample.gyro_body(1), imu_sample.gyro_body(2), imu_sample.gyro_body(3), ...
                        imu_sample.orientation(1), imu_sample.orientation(2), imu_sample.orientation(3));
                end
            end
        end

        % =================================================================
        % Standard empty IMU sample
        % =================================================================
        function imu_sample = makeEmptyImuSample(obj) %#ok<MANU>
            imu_sample = struct();
            imu_sample.accel_body = [NaN, NaN, NaN];
            imu_sample.gyro_body = [NaN, NaN, NaN];
            imu_sample.orientation = [NaN, NaN, NaN];
            imu_sample.linear_velocity_world = [NaN, NaN, NaN];
            imu_sample.accel_valid = false;
            imu_sample.gyro_valid = false;
            imu_sample.orientation_valid = false;
            imu_sample.timestamp = NaN;
            imu_sample.ros_timestamp = NaN;
            imu_sample.valid = false;
            imu_sample.source = "bebop";
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
            angles.source = "bebop";
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
        % Is latest /imu sample fresh?
        % =================================================================
        function fresh = isImuFresh(obj, timeout)
            if ~obj.HasImuSample || isnan(obj.LastImuReceiveWallTime)
                fresh = false;
                return;
            end
            age = obj.getWallTimeSeconds() - obj.LastImuReceiveWallTime;
            fresh = age <= timeout;
        end

        % =================================================================
        % Is latest /odom sample fresh?
        % =================================================================
        function fresh = isOdomFresh(obj, timeout)
            if ~obj.HasOdomSample || isnan(obj.LastOdomReceiveWallTime)
                fresh = false;
                return;
            end
            age = obj.getWallTimeSeconds() - obj.LastOdomReceiveWallTime;
            fresh = age <= timeout;
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
