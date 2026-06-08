% =========================================================================
% GENERALFILTER
% Author: Renzo Eisma
% Assistance note:
% ChatGPT Pro 5.5 Thinking Extended was used to clean up variable names,
% comments, line spacing, and general code structure.
% The original concept, code logic, and project structure were created by Renzo Eisma.
% Date: 06/2026
%
% Purpose:
% First filtering layer for UWB position output.
% This class receives a standard uwb_sample struct, filters the UWB XYZ
% position, logs useful debug data, and outputs a standard
% uwb_general_filtered struct.
%
% Standard input:
% uwb_sample.position  = [x y z]
% uwb_sample.quality   = optional quality value
% uwb_sample.timestamp = optional timestamp in seconds
% uwb_sample.source    = optional source string, e.g. 'python_udp'
% uwb_sample.valid     = true/false
%
% Standard output:
% uwb_general_filtered.position  = [x y z]
% uwb_general_filtered.velocity  = [vx vy vz]
% uwb_general_filtered.timestamp
% uwb_general_filtered.quality
% uwb_general_filtered.valid
% uwb_general_filtered.source    = 'general_filter'
%
% Design notes:
% - This class intentionally only handles calculated UWB XYZ positions.
% - IMU fusion belongs in ImuFusionFilter.m.
% - Raw-anchor-distance EKF logic belongs in the future custom PCB reader
%   or in a separate future filter, not here.
% - Minimal runaway fix: rejected samples hold the last accepted position instead of outputting prediction-only motion.

% =========================================================================

classdef GeneralFilter < handle

    % =====================================================================
    % USER CONFIGURATION
    % =====================================================================
    properties
        % -------------------------------------------------------------
        % Main filter switches
        % -------------------------------------------------------------
        % Kalman filtering
        USE_KALMAN_FILTER       = true; % Enables the Kalman filter, which estimates a smoother position and velocity based on the UWB measurements.
        % Outlier rejections
        USE_OUTLIER_REJECTION   = true; % Enables the following three outlier rejections
        USE_SPEED_GATE          = true; % Rejects measurements that would require the robot or tag to move faster than physically believable.
        USE_POSITION_JUMP_GATE  = true; % Rejects raw UWB measurements that suddenly jump too far compared to the previous raw measurement.
        USE_MAHALANOBIS_GATE    = false; % Rejects measurements that are too far away from the Kalman filter prediction based on the filter uncertainty.
        % Smoothing
        USE_LOW_PASS_OUTPUT     = false; % Applies optional low-pass smoothing to the final filtered output to reduce small remaining jitter.

        % -------------------------------------------------------------
        % Timing
        % -------------------------------------------------------------
        dt = 0.1;                         % default UWB update time [s]
        USE_SAMPLE_TIMESTAMPS = false;    % false keeps old behaviour
        MIN_DT = 0.001;                   % protects against zero dt
        MAX_DT = 1.0;                     % protects against old samples

        % -------------------------------------------------------------
        % Kalman tuning
        % Higher process noise = faster reaction, less smoothing
        % Higher measurement noise = more smoothing, more lag
        % -------------------------------------------------------------
        PROCESS_NOISE_POSITION = 0.03; % old = 0.05
        PROCESS_NOISE_VELOCITY = 0.75; % old = 1.0

        MEASUREMENT_NOISE_X = 0.5^2;      % expected UWB X noise [m^2]
        MEASUREMENT_NOISE_Y = 0.5^2;      % expected UWB Y noise [m^2]
        MEASUREMENT_NOISE_Z = 1.0^2; % old 0.7      % expected UWB Z noise [m^2]

        INITIAL_POSITION_UNCERTAINTY = 1.0;
        INITIAL_VELOCITY_UNCERTAINTY = 5.0;

        % -------------------------------------------------------------
        % Outlier rejection tuning
        % -------------------------------------------------------------
        MAX_ALLOWED_SPEED = 12.0; % old = 4.0          % max believable speed [m/s], 10.0 would actually be a difference of 100cm between two measurements because of dt = 0.1
        MAX_POSITION_JUMP = 1.0;          % max jump between raw samples [m]
        MAHALANOBIS_GATE  = 16.27; % old = 11.34        % approx. 99% gate for 3D
        MIN_QUALITY       = -Inf;         % set if quality has a clear scale

        % -------------------------------------------------------------
        % Anti-runaway tuning
        % -------------------------------------------------------------
        MAX_REJECTED_COAST_STEPS = 2;
        MAX_CONSECUTIVE_REJECTS = 2;          % reset if this many measurements are rejected in a row
        MAX_COAST_SPEED = 3.0;              % [m/s]
        PREDICT_ONLY_VELOCITY_DAMPING = 0.20; % reduce velocity when no measurement update is accepted
        FILTER_RAW_RESET_DISTANCE = 0.0;     % reset to raw UWB if prediction is this far from raw [m]

        % -------------------------------------------------------------
        % Optional low-pass output smoothing
        % Alpha close to 1 = fast response
        % Alpha close to 0 = heavy smoothing / more delay
        % -------------------------------------------------------------
        LOW_PASS_ALPHA = 0.85;

        % -------------------------------------------------------------
        % Simple debug logging
        % -------------------------------------------------------------
        LOG_TO_CSV = true;
        LOG_FILE_PATH = '';               % if empty, no file is opened
    end

    % =====================================================================
    % INTERNAL FILTER VARIABLES
    % =====================================================================
    properties
        IsInitialized = false;

        % Kalman state: [x; y; z; vx; vy; vz]
        X = [];
        P = [];

        % Kalman matrices
        F = [];
        H = [];
        Q = [];
        R = [];

        % Previous values
        LastRawMeasurement = [];
        LastOutputPosition = [];
        LastTimestamp = [];
        LastVelocity = [0; 0; 0];
        LastOutputStruct = struct();

        % Anti-runaway state
        ConsecutiveRejects = 0;
        LastAcceptedPosition = [];

        % Debug values
        OutlierCount = 0;
        AcceptedCount = 0;
        InvalidCount = 0;
        PredictionOnlyCount = 0;
        LastRejectionReason = 'none';
        LastMahalanobisDistance = NaN;
        LastImpliedSpeed = NaN;
        LastInnovation = [NaN; NaN; NaN];

        % Logging
        LogFileId = -1;
        LogHeaderWritten = false;
    end

    % =====================================================================
    % METHODS
    % =====================================================================
    methods

        % =================================================================
        % Constructor
        % =================================================================
        function obj = GeneralFilter(config_or_dt, log_file_path)
            if nargin >= 1
                if isnumeric(config_or_dt)
                    obj.dt = config_or_dt;
                elseif isstruct(config_or_dt)
                    obj.configure(config_or_dt);
                end
            end

            if nargin >= 2
                obj.LOG_FILE_PATH = log_file_path;
            end

            obj.updateMatrices(obj.dt);
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

            obj.updateMatrices(obj.dt);
        end

        % =================================================================
        % Main function: process standard uwb_sample struct
        % =================================================================
        function uwb_general_filtered = processUwbSample(obj, uwb_sample)
            [Z, quality, timestamp, source, valid_input] = obj.parseUwbSample(uwb_sample);

            obj.LastRejectionReason = 'none';
            obj.LastMahalanobisDistance = NaN;
            obj.LastImpliedSpeed = NaN;
            obj.LastInnovation = [NaN; NaN; NaN];

            if ~valid_input
                obj.InvalidCount = obj.InvalidCount + 1;

                uwb_general_filtered = obj.outputPredictionOnly( ...
                    timestamp, quality, source, false, 'invalid_input', Z);

                obj.logFilteredData(uwb_general_filtered);
                return;
            end

            current_dt = obj.calculateDt(timestamp);
            obj.updateMatrices(current_dt);

            % -------------------------------------------------------------
            % First measurement initializes the filter
            % -------------------------------------------------------------
            if ~obj.IsInitialized
                obj.initializeFilter(Z, timestamp);

                output_pos = obj.applyLowPass(Z);
                uwb_general_filtered = obj.makeOutputStruct( ...
                    output_pos, [0; 0; 0], timestamp, quality, true, true, ...
                    'initialized', Z, source);

                obj.AcceptedCount = obj.AcceptedCount + 1;
                obj.logFilteredData(uwb_general_filtered);
                return;
            end

            % -------------------------------------------------------------
            % If Kalman is disabled: only outlier rejection + low-pass
            % -------------------------------------------------------------
            if ~obj.USE_KALMAN_FILTER
                accepted = obj.simpleRawGate(Z, current_dt, quality);

                if accepted
                    output_pos = Z;
                    velocity = obj.calculateVelocityFromRaw(Z, current_dt);

                    obj.LastRawMeasurement = Z;
                    obj.LastVelocity = velocity;
                    obj.AcceptedCount = obj.AcceptedCount + 1;
                    reason = 'accepted_no_kalman';
                else
                    output_pos = obj.getSafeLastOutput();
                    velocity = obj.LastVelocity;

                    obj.OutlierCount = obj.OutlierCount + 1;
                    reason = obj.LastRejectionReason;
                end

                output_pos = obj.applyLowPass(output_pos);
                uwb_general_filtered = obj.makeOutputStruct( ...
                    output_pos, velocity, timestamp, quality, true, accepted, ...
                    reason, Z, source);

                obj.LastTimestamp = timestamp;
                obj.logFilteredData(uwb_general_filtered);
                return;
            end

            % -------------------------------------------------------------
            % Kalman predict step
            % -------------------------------------------------------------
            X_pred = obj.F * obj.X;
            P_pred = obj.F * obj.P * obj.F' + obj.Q;

            % -------------------------------------------------------------
            % Outlier rejection
            % -------------------------------------------------------------
            accepted = true;

            if obj.USE_OUTLIER_REJECTION
                accepted = obj.checkOutlierGates(Z, X_pred, P_pred, current_dt, quality);
            end

            % -------------------------------------------------------------
            % Kalman update or anti-runaway rejected-sample handling
            % -------------------------------------------------------------
            if accepted
                residual = Z - (obj.H * X_pred);
                S = obj.H * P_pred * obj.H' + obj.R;
                K = P_pred * obj.H' / S;

                obj.X = X_pred + (K * residual);
                obj.P = (eye(6) - K * obj.H) * P_pred;

                obj.LastRawMeasurement = Z;
                obj.AcceptedCount = obj.AcceptedCount + 1;

                % Anti-runaway state is reset after a successful measurement update.
                obj.ConsecutiveRejects = 0;
                obj.LastAcceptedPosition = obj.X(1:3);

                reason = 'accepted';
            else
                obj.OutlierCount = obj.OutlierCount + 1;
                obj.PredictionOnlyCount = obj.PredictionOnlyCount + 1;
                obj.ConsecutiveRejects = obj.ConsecutiveRejects + 1;
            
                % Distance between current raw UWB and the Kalman prediction.
                raw_distance_to_prediction = norm(Z - X_pred(1:3));
            
                % -------------------------------------------------------------
                % Reject 1 and reject 2:
                % Carefully keep predicting based on velocity.
                % -------------------------------------------------------------
                if obj.ConsecutiveRejects <= obj.MAX_REJECTED_COAST_STEPS
            
                    obj.X = X_pred;
                    obj.P = P_pred;
            
                    % Clamp velocity during coasting so the filter cannot run away.
                    coast_speed = norm(obj.X(4:6));
            
                    if coast_speed > obj.MAX_COAST_SPEED
                        obj.X(4:6) = obj.X(4:6) / coast_speed * obj.MAX_COAST_SPEED;
                    end
            
                    reason = [obj.LastRejectionReason '_coast'];
            
                % -------------------------------------------------------------
                % Reject 3:
                % Reset/relock to raw UWB if the raw measurement is far enough
                % from the prediction.
                % -------------------------------------------------------------
                elseif raw_distance_to_prediction >= obj.FILTER_RAW_RESET_DISTANCE
            
                    obj.X = [Z; 0; 0; 0];
            
                    obj.P = diag([
                        obj.INITIAL_POSITION_UNCERTAINTY;
                        obj.INITIAL_POSITION_UNCERTAINTY;
                        obj.INITIAL_POSITION_UNCERTAINTY;
                        obj.INITIAL_VELOCITY_UNCERTAINTY;
                        obj.INITIAL_VELOCITY_UNCERTAINTY;
                        obj.INITIAL_VELOCITY_UNCERTAINTY
                    ]);
            
                    obj.LastRawMeasurement = Z;
                    obj.LastAcceptedPosition = Z;
                    obj.ConsecutiveRejects = 0;
            
                    reason = [obj.LastRejectionReason '_reset_to_raw'];
            
                % -------------------------------------------------------------
                % Reject 3 but raw is still close:
                % Do not reset, just hold last accepted position.
                % This avoids resetting to tiny suspicious movements.
                % -------------------------------------------------------------
                else
            
                    if ~isempty(obj.LastAcceptedPosition)
                        obj.X(1:3) = obj.LastAcceptedPosition;
                    end
            
                    obj.X(4:6) = [0; 0; 0];
                    obj.P = P_pred;
            
                    reason = [obj.LastRejectionReason '_held_close'];
            
                end
            end

            output_pos = obj.X(1:3);
            output_vel = obj.X(4:6);

            output_pos = obj.applyLowPass(output_pos);

            uwb_general_filtered = obj.makeOutputStruct( ...
                output_pos, output_vel, timestamp, quality, true, accepted, ...
                reason, Z, source);

            obj.LastTimestamp = timestamp;
            obj.LastVelocity = output_vel;
            obj.logFilteredData(uwb_general_filtered);
        end

        % =================================================================
        % Legacy wrapper
        % Supports calls: [x,y,z] = filter.process(raw_x,raw_y,raw_z)
        % =================================================================
        function varargout = process(obj, raw_x, raw_y, raw_z)
            uwb_sample.position = [raw_x raw_y raw_z];
            uwb_sample.quality = NaN;
            uwb_sample.timestamp = obj.getCurrentTimestamp();
            uwb_sample.source = 'legacy_process';
            uwb_sample.valid = true;

            out = obj.processUwbSample(uwb_sample);

            if nargout >= 1
                varargout{1} = out.position(1);
            end
            if nargout >= 2
                varargout{2} = out.position(2);
            end
            if nargout >= 3
                varargout{3} = out.position(3);
            end
            if nargout >= 4
                varargout{4} = out;
            end
        end

        % =================================================================
        % Reset filter
        % =================================================================
        function reset(obj)
            obj.IsInitialized = false;
            obj.X = [];
            obj.P = [];

            obj.LastRawMeasurement = [];
            obj.LastOutputPosition = [];
            obj.LastTimestamp = [];
            obj.LastVelocity = [0; 0; 0];
            obj.LastOutputStruct = struct();

            obj.ConsecutiveRejects = 0;
            obj.LastAcceptedPosition = [];

            obj.OutlierCount = 0;
            obj.AcceptedCount = 0;
            obj.InvalidCount = 0;
            obj.PredictionOnlyCount = 0;
            obj.LastRejectionReason = 'none';
            obj.LastMahalanobisDistance = NaN;
            obj.LastImpliedSpeed = NaN;
            obj.LastInnovation = [NaN; NaN; NaN];

            obj.updateMatrices(obj.dt);
        end

        % =================================================================
        % Get current output without processing a new sample
        % =================================================================
        function output = getOutput(obj)
            if ~isempty(obj.LastOutputStruct)
                output = obj.LastOutputStruct;
            else
                output = obj.makeEmptyOutput();
            end
        end

        % =================================================================
        % Get useful debug information
        % =================================================================
        function debug = getDebugInfo(obj)
            debug.accepted_count = obj.AcceptedCount;
            debug.outlier_count = obj.OutlierCount;
            debug.invalid_count = obj.InvalidCount;
            debug.prediction_only_count = obj.PredictionOnlyCount;
            debug.last_rejection_reason = obj.LastRejectionReason;
            debug.last_mahalanobis_distance = obj.LastMahalanobisDistance;
            debug.last_implied_speed = obj.LastImpliedSpeed;
            debug.last_innovation = obj.LastInnovation(:)';
            debug.is_initialized = obj.IsInitialized;

            debug.consecutive_rejects = obj.ConsecutiveRejects;
        end

        % =================================================================
        % Set or change log file
        % =================================================================
        function setLogFile(obj, log_file_path)
            obj.closeLog();
            obj.LOG_FILE_PATH = log_file_path;
            obj.LogHeaderWritten = false;
            obj.openLogIfNeeded();
        end

        % =================================================================
        % Close log file manually
        % =================================================================
        function closeLog(obj)
            if obj.LogFileId > 0
                fclose(obj.LogFileId);
            end

            obj.LogFileId = -1;
        end
    end

    % =====================================================================
    % INTERNAL HELPER METHODS
    % =====================================================================
    methods (Access = private)

        % =================================================================
        % Update matrices from current tuning values
        % =================================================================
        function updateMatrices(obj, dt_val)
            if nargin < 2 || isempty(dt_val) || isnan(dt_val) || dt_val <= 0
                dt_val = obj.dt;
            end

            dt_val = max(obj.MIN_DT, min(obj.MAX_DT, dt_val));

            obj.F = [
                1 0 0 dt_val 0      0;
                0 1 0 0      dt_val 0;
                0 0 1 0      0      dt_val;
                0 0 0 1      0      0;
                0 0 0 0      1      0;
                0 0 0 0      0      1
            ];

            obj.H = [
                1 0 0 0 0 0;
                0 1 0 0 0 0;
                0 0 1 0 0 0
            ];

            obj.Q = diag([
                obj.PROCESS_NOISE_POSITION;
                obj.PROCESS_NOISE_POSITION;
                obj.PROCESS_NOISE_POSITION;
                obj.PROCESS_NOISE_VELOCITY;
                obj.PROCESS_NOISE_VELOCITY;
                obj.PROCESS_NOISE_VELOCITY
            ]);

            obj.R = diag([
                obj.MEASUREMENT_NOISE_X;
                obj.MEASUREMENT_NOISE_Y;
                obj.MEASUREMENT_NOISE_Z
            ]);
        end

        % =================================================================
        % Initialize filter state
        % =================================================================
        function initializeFilter(obj, Z, timestamp)
            obj.X = [Z; 0; 0; 0];
            obj.P = diag([
                obj.INITIAL_POSITION_UNCERTAINTY;
                obj.INITIAL_POSITION_UNCERTAINTY;
                obj.INITIAL_POSITION_UNCERTAINTY;
                obj.INITIAL_VELOCITY_UNCERTAINTY;
                obj.INITIAL_VELOCITY_UNCERTAINTY;
                obj.INITIAL_VELOCITY_UNCERTAINTY
            ]);

            obj.LastRawMeasurement = Z;
            obj.LastOutputPosition = Z;
            obj.LastTimestamp = timestamp;
            obj.LastVelocity = [0; 0; 0];
            obj.ConsecutiveRejects = 0;
            obj.LastAcceptedPosition = Z;
            obj.IsInitialized = true;
        end

        % =================================================================
        % Parse standard or partly-standard UWB sample
        % =================================================================
        function [Z, quality, timestamp, source, valid_input] = parseUwbSample(obj, uwb_sample)
            Z = [NaN; NaN; NaN];
            quality = NaN;
            timestamp = obj.getCurrentTimestamp();
            source = 'unknown';
            valid_input = false;

            if ~isstruct(uwb_sample)
                return;
            end

            if isfield(uwb_sample, 'position')
                pos = uwb_sample.position;

                if numel(pos) >= 3
                    Z = [pos(1); pos(2); pos(3)];
                end
            elseif all(isfield(uwb_sample, {'x','y','z'}))
                Z = [uwb_sample.x; uwb_sample.y; uwb_sample.z];
            end

            if isfield(uwb_sample, 'quality')
                quality = uwb_sample.quality;
            end

            if isfield(uwb_sample, 'timestamp') && ~isempty(uwb_sample.timestamp)
                timestamp = uwb_sample.timestamp;
            end

            if isfield(uwb_sample, 'source') && ~isempty(uwb_sample.source)
                source = char(string(uwb_sample.source));
            end

            sample_valid_flag = true;

            if isfield(uwb_sample, 'valid')
                sample_valid_flag = logical(uwb_sample.valid);
            end

            valid_input = sample_valid_flag && all(isfinite(Z));
        end

        % =================================================================
        % Calculate dt. By default this returns fixed dt for compatibility.
        % =================================================================
        function current_dt = calculateDt(obj, timestamp)
            current_dt = obj.dt;

            if obj.USE_SAMPLE_TIMESTAMPS && ~isempty(obj.LastTimestamp) && ...
                    isfinite(timestamp) && isfinite(obj.LastTimestamp)

                measured_dt = timestamp - obj.LastTimestamp;

                if isfinite(measured_dt) && measured_dt > 0
                    current_dt = max(obj.MIN_DT, min(obj.MAX_DT, measured_dt));
                end
            end
        end

        % =================================================================
        % Outlier rejection gates for Kalman mode
        % =================================================================
        function accepted = checkOutlierGates(obj, Z, X_pred, P_pred, current_dt, quality)
            accepted = true;
            predicted_pos = X_pred(1:3);

            if isfinite(quality) && quality < obj.MIN_QUALITY
                obj.LastRejectionReason = 'quality_too_low';
                accepted = false;
                return;
            end

            if obj.USE_SPEED_GATE
                movement = norm(Z - predicted_pos);
                implied_speed = movement / current_dt;
                obj.LastImpliedSpeed = implied_speed;

                if implied_speed > obj.MAX_ALLOWED_SPEED
                    obj.LastRejectionReason = 'speed_gate';
                    accepted = false;
                    return;
                end
            end

            if obj.USE_POSITION_JUMP_GATE && ~isempty(obj.LastRawMeasurement)
                jump = norm(Z - obj.LastRawMeasurement);

                if jump > obj.MAX_POSITION_JUMP
                    obj.LastRejectionReason = 'position_jump_gate';
                    accepted = false;
                    return;
                end
            end

            if obj.USE_MAHALANOBIS_GATE
                residual = Z - (obj.H * X_pred);
                S = obj.H * P_pred * obj.H' + obj.R;
                mahalanobis_distance = residual' / S * residual;

                obj.LastInnovation = residual;
                obj.LastMahalanobisDistance = mahalanobis_distance;

                if mahalanobis_distance > obj.MAHALANOBIS_GATE
                    obj.LastRejectionReason = 'mahalanobis_gate';
                    accepted = false;
                    return;
                end
            end
        end

        % =================================================================
        % Simple raw gate for when Kalman is disabled
        % =================================================================
        function accepted = simpleRawGate(obj, Z, current_dt, quality)
            accepted = true;

            if ~obj.USE_OUTLIER_REJECTION
                return;
            end

            if isfinite(quality) && quality < obj.MIN_QUALITY
                obj.LastRejectionReason = 'quality_too_low';
                accepted = false;
                return;
            end

            if isempty(obj.LastRawMeasurement)
                return;
            end

            if obj.USE_SPEED_GATE
                movement = norm(Z - obj.LastRawMeasurement);
                implied_speed = movement / current_dt;
                obj.LastImpliedSpeed = implied_speed;

                if implied_speed > obj.MAX_ALLOWED_SPEED
                    obj.LastRejectionReason = 'speed_gate';
                    accepted = false;
                    return;
                end
            end

            if obj.USE_POSITION_JUMP_GATE
                jump = norm(Z - obj.LastRawMeasurement);

                if jump > obj.MAX_POSITION_JUMP
                    obj.LastRejectionReason = 'position_jump_gate';
                    accepted = false;
                    return;
                end
            end
        end

        % =================================================================
        % Optional low-pass output smoothing
        % =================================================================
        function output_pos = applyLowPass(obj, input_pos)
            if ~obj.USE_LOW_PASS_OUTPUT || isempty(obj.LastOutputPosition)
                output_pos = input_pos;
                obj.LastOutputPosition = output_pos;
                return;
            end

            alpha = obj.LOW_PASS_ALPHA;
            output_pos = alpha * input_pos + (1 - alpha) * obj.LastOutputPosition;
            obj.LastOutputPosition = output_pos;
        end

        % =================================================================
        % Prediction-only output for invalid samples or missing samples
        % Anti-runaway version: do not keep moving forever without updates.
        % =================================================================
        function output = outputPredictionOnly(obj, timestamp, quality, source, valid, reason, raw_position)
            if obj.IsInitialized && obj.USE_KALMAN_FILTER
                obj.PredictionOnlyCount = obj.PredictionOnlyCount + 1;
                obj.ConsecutiveRejects = obj.ConsecutiveRejects + 1;

                % Do not keep predicting forward without measurement updates.
                obj.X(4:6) = obj.PREDICT_ONLY_VELOCITY_DAMPING * obj.X(4:6);

                if ~isempty(obj.LastAcceptedPosition)
                    obj.X(1:3) = obj.LastAcceptedPosition;
                elseif ~isempty(obj.LastOutputPosition)
                    obj.X(1:3) = obj.LastOutputPosition;
                end

                output_pos = obj.applyLowPass(obj.X(1:3));
                output_vel = obj.X(4:6);
            elseif ~isempty(obj.LastOutputPosition)
                output_pos = obj.LastOutputPosition;
                output_vel = obj.LastVelocity;
            else
                output_pos = [NaN; NaN; NaN];
                output_vel = [NaN; NaN; NaN];
            end

            output = obj.makeOutputStruct( ...
                output_pos, output_vel, timestamp, quality, valid, false, ...
                reason, raw_position, source);
        end

        % =================================================================
        % Calculate simple velocity when Kalman is disabled
        % =================================================================
        function velocity = calculateVelocityFromRaw(obj, Z, current_dt)
            if isempty(obj.LastRawMeasurement) || current_dt <= 0
                velocity = [0; 0; 0];
            else
                velocity = (Z - obj.LastRawMeasurement) / current_dt;
            end
        end

        % =================================================================
        % Safe fallback for output position
        % =================================================================
        function output_pos = getSafeLastOutput(obj)
            if ~isempty(obj.LastOutputPosition)
                output_pos = obj.LastOutputPosition;
            elseif ~isempty(obj.LastRawMeasurement)
                output_pos = obj.LastRawMeasurement;
            else
                output_pos = [NaN; NaN; NaN];
            end
        end

        % =================================================================
        % Create standard output struct
        % =================================================================
        function output = makeOutputStruct( ...
                obj, position, velocity, timestamp, quality, valid, accepted, ...
                reason, raw_position, input_source)

            output.position = position(:)';
            output.velocity = velocity(:)';
            output.timestamp = timestamp;
            output.quality = quality;
            output.valid = logical(valid);
            output.source = 'general_filter';
            output.raw_position = raw_position(:)';
            output.input_source = input_source;
            output.accepted = logical(accepted);
            output.rejection_reason = reason;

            output.debug.accepted_count = obj.AcceptedCount;
            output.debug.outlier_count = obj.OutlierCount;
            output.debug.invalid_count = obj.InvalidCount;
            output.debug.prediction_only_count = obj.PredictionOnlyCount;
            output.debug.mahalanobis_distance = obj.LastMahalanobisDistance;
            output.debug.implied_speed = obj.LastImpliedSpeed;
            output.debug.innovation = obj.LastInnovation(:)';
            output.debug.is_initialized = obj.IsInitialized;

            output.debug.consecutive_rejects = obj.ConsecutiveRejects;

            obj.LastOutputStruct = output;
        end

        % =================================================================
        % Empty output before the filter has started
        % =================================================================
        function output = makeEmptyOutput(obj)
            output.position = [NaN NaN NaN];
            output.velocity = [NaN NaN NaN];
            output.timestamp = obj.getCurrentTimestamp();
            output.quality = NaN;
            output.valid = false;
            output.source = 'general_filter';
            output.raw_position = [NaN NaN NaN];
            output.input_source = 'none';
            output.accepted = false;
            output.rejection_reason = 'not_initialized';
            output.debug = obj.getDebugInfo();
        end

        % =================================================================
        % Time helper. Uses POSIX seconds if possible.
        % =================================================================
        function t = getCurrentTimestamp(~)
            try
                t = posixtime(datetime('now'));
            catch
                t = now * 24 * 3600; %#ok<TNOW1> datenum days -> seconds
            end
        end

        % =================================================================
        % Open CSV log
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
        % Write one line of filtered UWB position for measurement reports
        % =================================================================
        function logFilteredData(obj, output)
            if ~obj.LOG_TO_CSV || obj.LogFileId <= 0
                return;
            end

            % -------------------------------------------------------------
            % Simple measurement-report format
            % -------------------------------------------------------------
            % The report maker expects only:
            % Time, POSX, POSY, POSZ
            %
            % POSX/POSY/POSZ are the filtered UWB coordinates.
            % -------------------------------------------------------------
            if ~obj.LogHeaderWritten
                fprintf(obj.LogFileId, 'Time,POSX,POSY,POSZ\n');
                obj.LogHeaderWritten = true;
            end

            fprintf(obj.LogFileId, '%.6f,%.6f,%.6f,%.6f\n', ...
                output.timestamp, ...
                output.position(1), ...
                output.position(2), ...
                output.position(3));

        end

        % =================================================================
        % Keep CSV text simple
        % =================================================================
        function txt = safeCsvText(~, value)
            txt = char(string(value));
            txt = strrep(txt, ',', '_');
            txt = strrep(txt, newline, ' ');
        end
    end
end
