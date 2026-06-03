% =========================================================================
% IMUFUSIONFILTER
% Author: Renzo Eisma / rewritten with ChatGPT
% Date: 06/2026
%
% Purpose:
%   Second localization layer after GeneralFilter.m.
%   This first version is intentionally simple and safe:
%       - accepts uwb_general_filtered from GeneralFilter.m
%       - accepts optional imu_sample from robot/custom PCB reader
%       - stores the latest IMU data and orientation
%       - outputs a standard uwb_imu_filtered struct
%       - if IMU data is missing or disabled, it falls back to the
%         GeneralFilter output
%
% Important:
%   This version does NOT do real acceleration prediction yet.
%   It is a pass-through/fallback structure so the master script can be
%   updated safely before real UWB+IMU fusion is added.
%
% Standard input 1:
%   uwb_general_filtered.position = [x y z]
%   uwb_general_filtered.velocity = [vx vy vz]
%   uwb_general_filtered.timestamp
%   uwb_general_filtered.quality
%   uwb_general_filtered.valid
%   uwb_general_filtered.source = 'general_filter'
%
% Standard input 2:
%   imu_sample.accel_body  = [ax ay az]
%   imu_sample.gyro_body   = [gx gy gz]
%   imu_sample.orientation = [roll pitch yaw]
%   imu_sample.timestamp
%   imu_sample.valid
%   imu_sample.source      = 'limo' / 'bebop' / 'custom_pcb'
%
% Standard output:
%   uwb_imu_filtered.position = [x y z]
%   uwb_imu_filtered.velocity = [vx vy vz]
%   uwb_imu_filtered.orientation = [roll pitch yaw]
%   uwb_imu_filtered.timestamp
%   uwb_imu_filtered.valid
%   uwb_imu_filtered.source = 'imu_fusion_filter'
% =========================================================================

classdef ImuFusionFilter < handle

    % =====================================================================
    % USER CONFIGURATION
    % =====================================================================
    properties
        % Main switch. For this first version, enabling this only stores IMU
        % data and includes it in the output. It does not yet predict motion.
        USE_IMU_FILTER = false;

        % Always keep this true for now. If IMU data is missing, the output
        % remains the GeneralFilter output instead of stopping.
        FALLBACK_TO_GENERAL_FILTER = true;

        % Future switches. Present now so the structure is ready, but they
        % are not used for real math in this pass-through version.
        USE_IMU_PREDICTION = false;              % future
        USE_TILT_COMPENSATION = false;           % future
        USE_LEVER_ARM_COMPENSATION = false;      % future

        % Robot mode for future behaviour.
        % Options: 'GROUND_2D' or 'DRONE_3D'
        ROBOT_MODE = 'GROUND_2D';

        % IMU freshness. If the newest IMU sample is older than this, it is
        % treated as unavailable.
        IMU_FRESH_TIMEOUT = 0.5;                 % [s]

        % Future tag offset from robot centre to UWB tag in robot body frame.
        % This will be used later for lever-arm compensation:
        % p_center = p_tag - R * tag_offset_body
        TAG_OFFSET_BODY = [0; 0; 0];             % [m]

        % Simple logging
        LOG_TO_CSV = true;
        LOG_FILE_PATH = '';
    end

    % =====================================================================
    % INTERNAL STATE
    % =====================================================================
    properties
        LatestImuSample = struct();
        LatestUwbGeneral = struct();
        LastOutputStruct = struct();

        HasImuSample = false;
        HasUwbSample = false;

        % Debug counters
        NumImuSamples = 0;
        NumUwbCorrections = 0;
        NumFallbacks = 0;
        NumInvalidInputs = 0;
        NumOutputs = 0;

        LastStatus = 'not_started';
        LastFallbackReason = 'none';
        LastUsedImu = false;

        % Logging
        LogFileId = -1;
        LogHeaderWritten = false;
    end

    % =====================================================================
    % PUBLIC METHODS
    % =====================================================================
    methods

        % =================================================================
        % Constructor
        % Usage options:
        %   f = ImuFusionFilter()
        %   f = ImuFusionFilter(config_struct)
        %   f = ImuFusionFilter(log_file_path)
        %   f = ImuFusionFilter(config_struct, log_file_path)
        % =================================================================
        function obj = ImuFusionFilter(config_or_log_file_path, log_file_path)
            if nargin >= 1
                if isstruct(config_or_log_file_path)
                    obj.configure(config_or_log_file_path);
                elseif ischar(config_or_log_file_path) || isstring(config_or_log_file_path)
                    obj.LOG_FILE_PATH = char(string(config_or_log_file_path));
                end
            end

            if nargin >= 2
                obj.LOG_FILE_PATH = char(string(log_file_path));
            end

            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestUwbGeneral = obj.makeEmptyUwbGeneralSample();
            obj.LastOutputStruct = obj.makeEmptyOutput();

            obj.openLogIfNeeded();
        end

        % =================================================================
        % Destructor
        % =================================================================
        function delete(obj)
            obj.closeLog();
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
        end

        % =================================================================
        % Store/process a new IMU sample.
        % No fusion is done here yet. The sample is only stored and returned.
        % =================================================================
        function imu_out = processImuSample(obj, imu_sample)
            imu_out = obj.parseImuSample(imu_sample);

            if imu_out.valid
                obj.LatestImuSample = imu_out;
                obj.HasImuSample = true;
                obj.NumImuSamples = obj.NumImuSamples + 1;
                obj.LastStatus = 'imu_sample_received';
            else
                obj.NumInvalidInputs = obj.NumInvalidInputs + 1;
                obj.LastStatus = 'invalid_imu_sample';
            end
        end

        % =================================================================
        % Main function for this first version.
        % It accepts GeneralFilter output and optionally an IMU sample.
        % Real acceleration prediction is not implemented yet.
        % =================================================================
        function uwb_imu_filtered = processUwbSample(obj, uwb_general_filtered, imu_sample)
            if nargin >= 3 && ~isempty(imu_sample)
                obj.processImuSample(imu_sample);
            end

            uwb = obj.parseUwbGeneralSample(uwb_general_filtered);
            obj.LatestUwbGeneral = uwb;
            obj.HasUwbSample = uwb.valid;

            obj.NumUwbCorrections = obj.NumUwbCorrections + 1;
            obj.LastFallbackReason = 'none';
            obj.LastUsedImu = false;

            if ~uwb.valid
                obj.NumInvalidInputs = obj.NumInvalidInputs + 1;
                obj.LastStatus = 'invalid_uwb_general_input';

                uwb_imu_filtered = obj.makeOutputStruct( ...
                    [NaN NaN NaN], [NaN NaN NaN], [NaN NaN NaN], NaN, false, ...
                    'invalid_uwb_general_input', false, false, uwb, obj.LatestImuSample);

                obj.LastOutputStruct = uwb_imu_filtered;
                obj.logFusedData(uwb_imu_filtered);
                return;
            end

            imu_available = obj.isImuAvailableForTimestamp(uwb.timestamp);

            % -------------------------------------------------------------
            % First safe version: always use GeneralFilter position as the
            % actual output. If IMU exists, attach orientation/debug only.
            % -------------------------------------------------------------
            if obj.USE_IMU_FILTER && imu_available
                orientation = obj.LatestImuSample.orientation;
                used_imu = true;
                fallback_used = false;
                status = 'pass_through_with_imu_available_no_prediction_yet';
            else
                orientation = [NaN NaN NaN];
                used_imu = false;
                fallback_used = true;

                if ~obj.USE_IMU_FILTER
                    status = 'fallback_imu_filter_disabled';
                    obj.LastFallbackReason = 'imu_filter_disabled';
                elseif ~imu_available
                    status = 'fallback_imu_missing_or_old';
                    obj.LastFallbackReason = 'imu_missing_or_old';
                else
                    status = 'fallback_unknown';
                    obj.LastFallbackReason = 'unknown';
                end

                obj.NumFallbacks = obj.NumFallbacks + 1;
            end

            % Future place for tag offset compensation:
            %   position = obj.applyLeverArmCompensation(uwb.position, orientation);
            % For now, keep exact GeneralFilter behaviour.
            position = uwb.position;
            velocity = uwb.velocity;

            uwb_imu_filtered = obj.makeOutputStruct( ...
                position, velocity, orientation, uwb.timestamp, uwb.valid, ...
                status, used_imu, fallback_used, uwb, obj.LatestImuSample);

            obj.LastOutputStruct = uwb_imu_filtered;
            obj.LastStatus = status;
            obj.LastUsedImu = used_imu;
            obj.NumOutputs = obj.NumOutputs + 1;

            obj.logFusedData(uwb_imu_filtered);
        end

        % =================================================================
        % Get latest output
        % =================================================================
        function output = getOutput(obj)
            if ~isempty(obj.LastOutputStruct)
                output = obj.LastOutputStruct;
            else
                output = obj.makeEmptyOutput();
            end
        end

        % =================================================================
        % Get latest stored IMU sample
        % =================================================================
        function imu_sample = getLatestImuSample(obj)
            if obj.HasImuSample
                imu_sample = obj.LatestImuSample;
            else
                imu_sample = obj.makeEmptyImuSample();
            end
        end

        % =================================================================
        % Get useful debug information
        % =================================================================
        function debug = getDebugInfo(obj)
            debug.num_imu_samples = obj.NumImuSamples;
            debug.num_uwb_corrections = obj.NumUwbCorrections;
            debug.num_fallbacks = obj.NumFallbacks;
            debug.num_invalid_inputs = obj.NumInvalidInputs;
            debug.num_outputs = obj.NumOutputs;
            debug.last_status = obj.LastStatus;
            debug.last_fallback_reason = obj.LastFallbackReason;
            debug.last_used_imu = obj.LastUsedImu;
            debug.has_imu_sample = obj.HasImuSample;
            debug.has_uwb_sample = obj.HasUwbSample;
        end

        % =================================================================
        % Reset filter state
        % =================================================================
        function reset(obj)
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestUwbGeneral = obj.makeEmptyUwbGeneralSample();
            obj.LastOutputStruct = obj.makeEmptyOutput();

            obj.HasImuSample = false;
            obj.HasUwbSample = false;

            obj.NumImuSamples = 0;
            obj.NumUwbCorrections = 0;
            obj.NumFallbacks = 0;
            obj.NumInvalidInputs = 0;
            obj.NumOutputs = 0;

            obj.LastStatus = 'reset';
            obj.LastFallbackReason = 'none';
            obj.LastUsedImu = false;
        end

        % =================================================================
        % Change log file
        % =================================================================
        function setLogFile(obj, log_file_path)
            obj.closeLog();
            obj.LOG_FILE_PATH = char(string(log_file_path));
            obj.LogHeaderWritten = false;
            obj.openLogIfNeeded();
        end

        % =================================================================
        % Close CSV log
        % =================================================================
        function closeLog(obj)
            if obj.LogFileId > 0
                fclose(obj.LogFileId);
            end
            obj.LogFileId = -1;
        end
    end

    % =====================================================================
    % PRIVATE HELPER METHODS
    % =====================================================================
    methods (Access = private)

        % =================================================================
        % Parse GeneralFilter output into expected format
        % =================================================================
        function out = parseUwbGeneralSample(obj, input)
            out = obj.makeEmptyUwbGeneralSample();

            if ~isstruct(input)
                return;
            end

            if isfield(input, 'position') && numel(input.position) >= 3
                out.position = reshape(input.position(1:3), 1, 3);
            end

            if isfield(input, 'velocity') && numel(input.velocity) >= 3
                out.velocity = reshape(input.velocity(1:3), 1, 3);
            end

            if isfield(input, 'timestamp') && ~isempty(input.timestamp)
                out.timestamp = input.timestamp;
            else
                out.timestamp = obj.getCurrentTimestamp();
            end

            if isfield(input, 'quality')
                out.quality = input.quality;
            end

            if isfield(input, 'valid')
                out.valid = logical(input.valid);
            else
                out.valid = all(isfinite(out.position));
            end

            if isfield(input, 'source') && ~isempty(input.source)
                out.source = char(string(input.source));
            else
                out.source = 'general_filter';
            end

            if isfield(input, 'accepted')
                out.accepted = logical(input.accepted);
            end

            if isfield(input, 'rejection_reason')
                out.rejection_reason = char(string(input.rejection_reason));
            end

            if isfield(input, 'raw_position') && numel(input.raw_position) >= 3
                out.raw_position = reshape(input.raw_position(1:3), 1, 3);
            end

            if isfield(input, 'debug')
                out.debug = input.debug;
            end

            out.valid = out.valid && all(isfinite(out.position));
        end

        % =================================================================
        % Parse standard IMU sample
        % =================================================================
        function imu = parseImuSample(obj, input)
            imu = obj.makeEmptyImuSample();

            if ~isstruct(input)
                return;
            end

            if isfield(input, 'accel_body') && numel(input.accel_body) >= 3
                imu.accel_body = reshape(input.accel_body(1:3), 1, 3);
            elseif all(isfield(input, {'ax','ay','az'}))
                imu.accel_body = [input.ax, input.ay, input.az];
            end

            if isfield(input, 'gyro_body') && numel(input.gyro_body) >= 3
                imu.gyro_body = reshape(input.gyro_body(1:3), 1, 3);
            elseif all(isfield(input, {'gx','gy','gz'}))
                imu.gyro_body = [input.gx, input.gy, input.gz];
            end

            if isfield(input, 'orientation') && numel(input.orientation) >= 3
                imu.orientation = reshape(input.orientation(1:3), 1, 3);
            elseif all(isfield(input, {'roll','pitch','yaw'}))
                imu.orientation = [input.roll, input.pitch, input.yaw];
            end

            if isfield(input, 'timestamp') && ~isempty(input.timestamp)
                imu.timestamp = input.timestamp;
            else
                imu.timestamp = obj.getCurrentTimestamp();
            end

            if isfield(input, 'source') && ~isempty(input.source)
                imu.source = char(string(input.source));
            else
                imu.source = 'unknown_imu';
            end

            if isfield(input, 'valid')
                imu.valid = logical(input.valid);
            else
                % For this pass-through version, orientation is optional.
                % Accel/gyro may be NaN until robot readers are implemented.
                imu.valid = true;
            end

            % Require at least one useful vector to avoid treating empty
            % structs as real IMU data.
            has_some_data = any(isfinite(imu.accel_body)) || any(isfinite(imu.gyro_body)) || any(isfinite(imu.orientation));
            imu.valid = imu.valid && has_some_data && isfinite(imu.timestamp);
        end

        % =================================================================
        % Check whether stored IMU data is usable for the UWB timestamp
        % =================================================================
        function available = isImuAvailableForTimestamp(obj, uwb_timestamp)
            available = false;

            if ~obj.HasImuSample || ~obj.LatestImuSample.valid
                return;
            end

            if ~isfinite(uwb_timestamp) || ~isfinite(obj.LatestImuSample.timestamp)
                return;
            end

            age = abs(uwb_timestamp - obj.LatestImuSample.timestamp);
            available = age <= obj.IMU_FRESH_TIMEOUT;
        end

        % =================================================================
        % Create output struct
        % =================================================================
        function output = makeOutputStruct(obj, position, velocity, orientation, timestamp, valid, status, used_imu, fallback_used, uwb, imu)
            output.position = reshape(position(1:3), 1, 3);
            output.velocity = reshape(velocity(1:3), 1, 3);
            output.orientation = reshape(orientation(1:3), 1, 3);
            output.timestamp = timestamp;
            output.valid = logical(valid);
            output.source = 'imu_fusion_filter';

            output.quality = uwb.quality;
            output.used_imu = logical(used_imu);
            output.fallback_used = logical(fallback_used);
            output.status = char(string(status));

            output.input_uwb_source = uwb.source;
            output.input_uwb_valid = uwb.valid;
            output.input_uwb_accepted = uwb.accepted;
            output.input_uwb_rejection_reason = uwb.rejection_reason;

            output.imu_valid = imu.valid;
            output.imu_source = imu.source;
            output.imu_timestamp = imu.timestamp;
            output.accel_body = imu.accel_body;
            output.gyro_body = imu.gyro_body;

            output.debug = obj.getDebugInfo();
        end

        % =================================================================
        % Empty GeneralFilter sample
        % =================================================================
        function out = makeEmptyUwbGeneralSample(~)
            out.position = [NaN NaN NaN];
            out.velocity = [NaN NaN NaN];
            out.timestamp = NaN;
            out.quality = NaN;
            out.valid = false;
            out.source = 'general_filter';
            out.accepted = false;
            out.rejection_reason = 'none';
            out.raw_position = [NaN NaN NaN];
            out.debug = struct();
        end

        % =================================================================
        % Empty IMU sample
        % =================================================================
        function imu = makeEmptyImuSample(~)
            imu.accel_body = [NaN NaN NaN];
            imu.gyro_body = [NaN NaN NaN];
            imu.orientation = [NaN NaN NaN];
            imu.timestamp = NaN;
            imu.valid = false;
            imu.source = 'none';
        end

        % =================================================================
        % Empty output
        % =================================================================
        function output = makeEmptyOutput(obj)
            uwb = obj.makeEmptyUwbGeneralSample();
            imu = obj.makeEmptyImuSample();
            output = obj.makeOutputStruct([NaN NaN NaN], [NaN NaN NaN], [NaN NaN NaN], NaN, false, ...
                'not_initialized', false, true, uwb, imu);
        end

        % =================================================================
        % Open CSV log if configured
        % =================================================================
        function openLogIfNeeded(obj)
            if ~obj.LOG_TO_CSV || isempty(obj.LOG_FILE_PATH)
                return;
            end

            [folder, ~, ~] = fileparts(obj.LOG_FILE_PATH);
            if ~isempty(folder) && ~exist(folder, 'dir')
                mkdir(folder);
            end

            obj.LogFileId = fopen(obj.LOG_FILE_PATH, 'w');
            obj.LogHeaderWritten = false;
        end

        % =================================================================
        % Write one fused output line
        % =================================================================
        function logFusedData(obj, output)
            if ~obj.LOG_TO_CSV || obj.LogFileId <= 0
                return;
            end

            if ~obj.LogHeaderWritten
                fprintf(obj.LogFileId, ['timestamp,status,position_x,position_y,position_z,', ...
                    'velocity_x,velocity_y,velocity_z,roll,pitch,yaw,quality,valid,', ...
                    'used_imu,fallback_used,imu_valid,imu_source,input_uwb_source,', ...
                    'input_uwb_valid,input_uwb_accepted,input_uwb_rejection_reason,', ...
                    'accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,num_fallbacks\n']);
                obj.LogHeaderWritten = true;
            end

            fprintf(obj.LogFileId, ['%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,', ...
                '%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%s,%s,%d,%d,%s,', ...
                '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n'], ...
                output.timestamp, ...
                obj.safeCsvText(output.status), ...
                output.position(1), output.position(2), output.position(3), ...
                output.velocity(1), output.velocity(2), output.velocity(3), ...
                output.orientation(1), output.orientation(2), output.orientation(3), ...
                output.quality, ...
                output.valid, ...
                output.used_imu, ...
                output.fallback_used, ...
                output.imu_valid, ...
                obj.safeCsvText(output.imu_source), ...
                obj.safeCsvText(output.input_uwb_source), ...
                output.input_uwb_valid, ...
                output.input_uwb_accepted, ...
                obj.safeCsvText(output.input_uwb_rejection_reason), ...
                output.accel_body(1), output.accel_body(2), output.accel_body(3), ...
                output.gyro_body(1), output.gyro_body(2), output.gyro_body(3), ...
                output.debug.num_fallbacks);
        end

        % =================================================================
        % Time helper. Uses POSIX seconds if possible.
        % =================================================================
        function t = getCurrentTimestamp(~)
            try
                t = posixtime(datetime('now'));
            catch
                t = now * 24 * 3600; %#ok<TNOW1>
            end
        end

        % =================================================================
        % CSV text helper
        % =================================================================
        function txt = safeCsvText(~, value)
            txt = char(string(value));
            txt = strrep(txt, ',', '_');
            txt = strrep(txt, newline, ' ');
        end
    end
end
