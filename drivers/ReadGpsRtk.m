% =========================================================================
% READGPSRTK
% Author: Renzo Eisma / rewritten with ChatGPT
% Date: 06/2026
%
% Purpose:
%   Placeholder ROS reader for future GPS RTK outdoor ground truth.
%   For now this file is intentionally simple. It creates a standard GPS RTK
%   data path without forcing the rest of the MATLAB code to know the final
%   GPS RTK ROS topic names yet.
%
% Expected future use:
%   - Read GPS RTK data from ROS.
%   - Convert latitude/longitude/altitude to a local XYZ frame.
%   - Log the GPS RTK ground truth in a standard CSV format.
%   - Use the logged CSV later in the measurement report system.
%
% Standard gps_sample output:
%   gps_sample.position  = [x y z]       local frame [m], placeholder for now
%   gps_sample.latitude
%   gps_sample.longitude
%   gps_sample.altitude
%   gps_sample.timestamp
%   gps_sample.valid
%   gps_sample.source    = 'gps_rtk_ros'
%
% Notes:
%   - AUTO_CONNECT is false by default because this is a future placeholder.
%   - The coordinate conversion is intentionally basic. Later, when the GPS
%     RTK setup is known, convertGpsToLocalFrame() should be updated.
% =========================================================================

classdef ReadGpsRtk < handle

    % =====================================================================
    % USER CONFIGURATION
    % =====================================================================
    properties
        % Placeholder ROS namespace and topics.
        ROS_NAMESPACE = "/gps_rtk";

        % Preferred future topic: sensor_msgs/NavSatFix.
        FIX_TOPIC = "/gps_rtk/fix";
        FIX_MSG_TYPE = "sensor_msgs/NavSatFix";

        % Optional future topic if another group already publishes local XYZ.
        % Supported types in this placeholder:
        %   geometry_msgs/PoseStamped
        %   geometry_msgs/PointStamped
        %   nav_msgs/Odometry
        LOCAL_POSITION_TOPIC = "/gps_rtk/local_position";
        LOCAL_POSITION_MSG_TYPE = "geometry_msgs/PoseStamped";

        USE_LOCAL_POSITION_TOPIC = false;

        % Local origin for converting lat/lon/alt to local coordinates.
        % Fill these in during outdoor testing.
        ORIGIN_LATITUDE = NaN;     % [deg]
        ORIGIN_LONGITUDE = NaN;    % [deg]
        ORIGIN_ALTITUDE = NaN;     % [m]

        % Optional logging.
        LOG_ENABLE = true;
        GPS_LOG_FILE = "";

        % Do not auto-connect yet. The GPS RTK setup is future work.
        AUTO_CONNECT = false;

        % Data freshness timeout.
        DATA_FRESH_TIMEOUT = 1.0;  % [s]

        % Print debug every N samples. Set to Inf to disable.
        DEBUG_PRINT_EVERY = Inf;
    end

    % =====================================================================
    % INTERNAL STATE
    % =====================================================================
    properties
        fix_sub = [];
        local_position_sub = [];

        IsConnected = false;
        HasGpsSample = false;
        HasLocalPositionSample = false;

        LatestGpsSample = struct();
        NumGpsSamples = 0;

        LastReceiveWallTime = NaN;
        LastStatus = "not_started";
    end

    % =====================================================================
    % PUBLIC METHODS
    % =====================================================================
    methods

        % =================================================================
        % Constructor
        % Usage:
        %   reader = ReadGpsRtk()
        %   reader = ReadGpsRtk(config_struct)
        % =================================================================
        function obj = ReadGpsRtk(config)
            obj.LatestGpsSample = obj.makeEmptyGpsSample();

            if nargin >= 1 && isstruct(config)
                obj.configure(config);
            end

            obj.initLog();

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
                if ~isfield(config, 'FIX_TOPIC')
                    obj.FIX_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "fix");
                end
                if ~isfield(config, 'LOCAL_POSITION_TOPIC')
                    obj.LOCAL_POSITION_TOPIC = obj.buildTopic(obj.ROS_NAMESPACE, "local_position");
                end
            end

            obj.initLog();
        end

        % =================================================================
        % Connect to GPS RTK ROS topics
        % =================================================================
        function success = connect(obj)
            success = false;
            fix_ok = false;
            local_ok = false;

            try
                fprintf('[ReadGpsRtk] Subscribing to %s (%s)...\n', char(obj.FIX_TOPIC), char(obj.FIX_MSG_TYPE));
                obj.fix_sub = rossubscriber(char(obj.FIX_TOPIC), char(obj.FIX_MSG_TYPE), @obj.fixCallback);
                fix_ok = true;
            catch ME
                fprintf('[ReadGpsRtk] Could not subscribe to GPS fix topic: %s\n', ME.message);
            end

            if obj.USE_LOCAL_POSITION_TOPIC
                try
                    fprintf('[ReadGpsRtk] Subscribing to %s (%s)...\n', char(obj.LOCAL_POSITION_TOPIC), char(obj.LOCAL_POSITION_MSG_TYPE));
                    obj.local_position_sub = rossubscriber(char(obj.LOCAL_POSITION_TOPIC), char(obj.LOCAL_POSITION_MSG_TYPE), @obj.localPositionCallback);
                    local_ok = true;
                catch ME
                    fprintf('[ReadGpsRtk] Could not subscribe to local GPS position topic: %s\n', ME.message);
                end
            end

            obj.IsConnected = fix_ok || local_ok;
            success = obj.IsConnected;

            if success
                obj.LastStatus = "connected";
            else
                obj.LastStatus = "connect_failed";
                fprintf('[ReadGpsRtk] This is expected if GPS RTK ROS topics are not available yet.\n');
            end
        end

        % =================================================================
        % Disconnect / clear subscribers
        % =================================================================
        function disconnect(obj)
            try
                obj.fix_sub = [];
                obj.local_position_sub = [];
            catch
            end
            obj.IsConnected = false;
            obj.LastStatus = "disconnected";
        end

        % =================================================================
        % Return latest standard GPS sample
        % =================================================================
        function gps_sample = getGpsSample(obj)
            gps_sample = obj.LatestGpsSample;

            if ~obj.HasGpsSample || ~obj.isDataFresh()
                gps_sample.valid = false;
                if ~obj.HasGpsSample
                    gps_sample.status = "no_gps_sample_yet";
                else
                    gps_sample.status = "stale_gps_sample";
                end
            end
        end

        % =================================================================
        % Check whether newest sample is fresh enough
        % =================================================================
        function fresh = isDataFresh(obj, timeout)
            if nargin < 2 || isempty(timeout)
                timeout = obj.DATA_FRESH_TIMEOUT;
            end
            fresh = obj.HasGpsSample && ~isnan(obj.LastReceiveWallTime) && ...
                    (obj.getWallTimeSeconds() - obj.LastReceiveWallTime <= timeout);
        end

        % =================================================================
        % Set local origin from the current GPS sample
        % =================================================================
        function setOriginFromCurrentSample(obj)
            sample = obj.LatestGpsSample;
            if sample.valid && isfinite(sample.latitude) && isfinite(sample.longitude)
                obj.ORIGIN_LATITUDE = sample.latitude;
                obj.ORIGIN_LONGITUDE = sample.longitude;
                obj.ORIGIN_ALTITUDE = sample.altitude;
                fprintf('[ReadGpsRtk] Origin set to lat=%.8f lon=%.8f alt=%.3f\n', ...
                    obj.ORIGIN_LATITUDE, obj.ORIGIN_LONGITUDE, obj.ORIGIN_ALTITUDE);
            else
                fprintf('[ReadGpsRtk] Could not set origin: no valid GPS sample.\n');
            end
        end

        % =================================================================
        % Optional helper for manual tests without ROS
        % =================================================================
        function setFakeGpsSample(obj, latitude, longitude, altitude, timestamp)
            if nargin < 5 || isempty(timestamp)
                timestamp = obj.getWallTimeSeconds();
            end

            sample = obj.makeEmptyGpsSample();
            sample.latitude = latitude;
            sample.longitude = longitude;
            sample.altitude = altitude;
            sample.timestamp = timestamp;
            sample.position = obj.convertGpsToLocalFrame(latitude, longitude, altitude);
            sample.valid = true;
            sample.status = "fake_sample";

            obj.LatestGpsSample = sample;
            obj.HasGpsSample = true;
            obj.NumGpsSamples = obj.NumGpsSamples + 1;
            obj.LastReceiveWallTime = obj.getWallTimeSeconds();
            obj.logGpsSample(sample);
        end

        % =================================================================
        % Convert GPS to local XYZ placeholder
        % =================================================================
        function position = convertGpsToLocalFrame(obj, latitude, longitude, altitude)
            % Simple local tangent plane approximation.
            % Good enough as a placeholder for small outdoor test areas.
            % Later this can be replaced by a proper ENU conversion or by
            % using the GPS RTK system's own local frame if it provides one.

            position = [NaN NaN NaN];

            if any(~isfinite([latitude, longitude, altitude, obj.ORIGIN_LATITUDE, obj.ORIGIN_LONGITUDE, obj.ORIGIN_ALTITUDE]))
                return;
            end

            earth_radius = 6378137.0; % [m]
            lat0 = deg2rad(obj.ORIGIN_LATITUDE);

            dlat = deg2rad(latitude - obj.ORIGIN_LATITUDE);
            dlon = deg2rad(longitude - obj.ORIGIN_LONGITUDE);

            x = earth_radius * cos(lat0) * dlon;
            y = earth_radius * dlat;
            z = altitude - obj.ORIGIN_ALTITUDE;

            position = [x y z];
        end
    end

    % =====================================================================
    % CALLBACKS
    % =====================================================================
    methods (Access = private)

        % =================================================================
        % sensor_msgs/NavSatFix callback
        % =================================================================
        function fixCallback(obj, ~, msg)
            sample = obj.makeEmptyGpsSample();
            sample.timestamp = obj.extractTimestamp(msg);
            sample.latitude = double(msg.Latitude);
            sample.longitude = double(msg.Longitude);
            sample.altitude = double(msg.Altitude);
            sample.position = obj.convertGpsToLocalFrame(sample.latitude, sample.longitude, sample.altitude);
            sample.valid = isfinite(sample.latitude) && isfinite(sample.longitude) && isfinite(sample.altitude);

            try
                sample.fix_status = double(msg.Status.Status);
            catch
                sample.fix_status = NaN;
            end

            if sample.valid
                if all(isfinite(sample.position))
                    sample.status = "ok_local_converted";
                else
                    sample.status = "ok_no_local_origin";
                end
            else
                sample.status = "invalid_fix";
            end

            obj.LatestGpsSample = sample;
            obj.HasGpsSample = true;
            obj.NumGpsSamples = obj.NumGpsSamples + 1;
            obj.LastReceiveWallTime = obj.getWallTimeSeconds();
            obj.logGpsSample(sample);

            if isfinite(obj.DEBUG_PRINT_EVERY) && obj.DEBUG_PRINT_EVERY > 0 && ...
                    mod(obj.NumGpsSamples, obj.DEBUG_PRINT_EVERY) == 0
                fprintf('[ReadGpsRtk] GPS: lat=%.8f lon=%.8f alt=%.3f valid=%d\n', ...
                    sample.latitude, sample.longitude, sample.altitude, sample.valid);
            end
        end

        % =================================================================
        % Optional local position callback
        % =================================================================
        function localPositionCallback(obj, ~, msg)
            sample = obj.LatestGpsSample;
            if isempty(fieldnames(sample))
                sample = obj.makeEmptyGpsSample();
            end

            sample.timestamp = obj.extractTimestamp(msg);
            sample.position = obj.extractLocalPosition(msg);
            sample.valid = all(isfinite(sample.position));
            sample.status = "local_position_topic";

            obj.LatestGpsSample = sample;
            obj.HasGpsSample = true;
            obj.HasLocalPositionSample = true;
            obj.NumGpsSamples = obj.NumGpsSamples + 1;
            obj.LastReceiveWallTime = obj.getWallTimeSeconds();
            obj.logGpsSample(sample);
        end
    end

    % =====================================================================
    % HELPER METHODS
    % =====================================================================
    methods (Access = private)

        function sample = makeEmptyGpsSample(obj) %#ok<MANU>
            sample = struct();
            sample.position = [NaN NaN NaN];
            sample.latitude = NaN;
            sample.longitude = NaN;
            sample.altitude = NaN;
            sample.fix_status = NaN;
            sample.timestamp = NaN;
            sample.valid = false;
            sample.source = "gps_rtk_ros";
            sample.status = "empty";
        end

        function initLog(obj)
            if ~obj.LOG_ENABLE || strlength(string(obj.GPS_LOG_FILE)) == 0
                return;
            end

            if exist(char(obj.GPS_LOG_FILE), 'file') ~= 2
                fid = fopen(char(obj.GPS_LOG_FILE), 'w');
                if fid >= 0
                    fprintf(fid, 'timestamp,x,y,z,latitude,longitude,altitude,fix_status,valid,source,status\n');
                    fclose(fid);
                end
            end
        end

        function logGpsSample(obj, sample)
            if ~obj.LOG_ENABLE || strlength(string(obj.GPS_LOG_FILE)) == 0
                return;
            end

            fid = fopen(char(obj.GPS_LOG_FILE), 'a');
            if fid < 0
                return;
            end

            fprintf(fid, '%.6f,%.6f,%.6f,%.6f,%.10f,%.10f,%.6f,%.0f,%d,%s,%s\n', ...
                sample.timestamp, sample.position(1), sample.position(2), sample.position(3), ...
                sample.latitude, sample.longitude, sample.altitude, sample.fix_status, ...
                sample.valid, char(sample.source), char(sample.status));
            fclose(fid);
        end

        function position = extractLocalPosition(obj, msg) %#ok<INUSL>
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
end
