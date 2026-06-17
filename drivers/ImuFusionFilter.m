% =========================================================================
% IMUFUSIONFILTER
% =========================================================================
% Author: Renzo Eisma
% Date: 06/2026
% Assistance note:
% ChatGPT Pro 5.5 Thinking Extended was used to write a big part of the 
% code. Minimal work was done by hand. The final version was checked by a
% human but the script has not been fully tested.
%
% Purpose:
% Loosely coupled UWB + IMU fusion layer after GeneralFilter.m.
% - UWB position from GeneralFilter is used as the correction source.
% - IMU acceleration is used for short-term prediction between UWB updates.
% - Orientation is used for tilt compensation during IMU prediction.
%
% Process loop summary: 
% 1. GeneralFilter.m first filters the calculated UWB XYZ position. 
% 2. ImuFusionFilter.m receives this filtered UWB position as an absolute 
% correction source. 
% 3. When fresh IMU data is available, acceleration is rotated from the IMU 
% body frame to the world frame using roll/pitch/yaw and is used for 
% short-term prediction between UWB updates. 
% 4. Every new UWB sample corrects the predicted state again, preventing 
% long-term IMU drift. 
% 5. If no valid or fresh IMU data is available, the filter falls back to the 
% GeneralFilter UWB output instead of crashing.
%
% Notes:
% - This is loosely coupled UWB/IMU fusion, not tightly coupled fusion. 
% Tightly coupled fusion would use the individual UWB anchor distances 
% directly together with IMU data. 
% - This is a linear Kalman filter, not an Extended Kalman Filter. The state 
% is [x y z vx vy vz], and the UWB correction is a direct XYZ position 
% update. Extended kalman filter can be used when raw anchor to tag 
% distances are available.
%
% Inputs and Outputs:
% - Standard input 1:
%       uwb_general_filtered.position = [x y z]
%       uwb_general_filtered.velocity = [vx vy vz]
%       uwb_general_filtered.timestamp
%       uwb_general_filtered.quality
%       uwb_general_filtered.valid
%       uwb_general_filtered.source = 'general_filter'
%
% - Standard input 2:
%       imu_sample.accel_body = [ax ay az]
%       imu_sample.gyro_body = [gx gy gz]
%       imu_sample.orientation = [roll pitch yaw] in degrees
%       imu_sample.timestamp
%       imu_sample.valid
%       imu_sample.source = 'limo' / 'bebop' / 'custom_pcb'
%
% - Standard output:
%       uwb_imu_filtered.position = [x y z]
%       uwb_imu_filtered.velocity = [vx vy vz]
%       uwb_imu_filtered.orientation = [roll pitch yaw] in degrees
%       uwb_imu_filtered.timestamp
%       uwb_imu_filtered.valid
%       uwb_imu_filtered.source = 'imu_fusion_filter'
%
% Future work:
% - Implement quality factor from UWB for the R in the kalman filter
% =========================================================================

classdef ImuFusionFilter < handle

    % =====================================================================
    % USER CONFIGURATION
    % =====================================================================
    properties
        % -------------------------------------------------------------
        % Main switches
        % -------------------------------------------------------------
        USE_IMU_FILTER = false;          % Enables IMU-based prediction. False = mostly follows GeneralFilter.
        FALLBACK_TO_GENERAL_FILTER = true; % If IMU is missing/old, mark output as fallback to GeneralFilter.
    
        USE_IMU_PREDICTION = true;       % Uses acceleration to predict position/velocity between UWB updates.
        USE_TILT_COMPENSATION = true;    % Uses orientation to rotate IMU body acceleration to world frame.
    
        % Options: 'GROUND_2D' or 'DRONE_3D'
        ROBOT_MODE = 'DRONE_3D';        % GROUND_2D forces vertical velocity/acceleration to zero.
    
        % -------------------------------------------------------------
        % Timing
        % -------------------------------------------------------------
        IMU_FRESH_TIMEOUT = 0.5;         % Maximum allowed age difference between UWB and IMU sample [s].
        MAX_IMU_DT = 0.10;               % Maximum integration step for one IMU prediction [s].
    
        % -------------------------------------------------------------
        % Acceleration handling
        % -------------------------------------------------------------
        GRAVITY = 9.81;                  % Gravity magnitude [m/s^2], world frame Z-up.
        REMOVE_GRAVITY = true;           % Subtracts [0 0 9.81] after rotating acceleration to world frame.
        ACCEL_BIAS_BODY = [0; 0; 0];     % Constant accelerometer bias in body frame [m/s^2].
    
        % -------------------------------------------------------------
        % Kalman tuning
        % -------------------------------------------------------------
        % These are empirical standard-deviation-like tuning values.
        PROCESS_NOISE_POSITION = 0.02;   % Position process noise tuning [m].
        PROCESS_NOISE_VELOCITY = 0.50;   % Velocity process noise tuning [m/s].
        MEASUREMENT_NOISE_POSITION = [0.08; 0.08; 0.10]; % UWB correction noise std [m].
    
        % -------------------------------------------------------------
        % UWB velocity handling
        % -------------------------------------------------------------
        USE_UWB_VELOCITY_INIT = true;    % Initializes velocity from GeneralFilter if available.
        UWB_VELOCITY_BLEND = 0.20;       % Blends GeneralFilter velocity into fused velocity on UWB correction.
    
        % -------------------------------------------------------------
        % CSV logging
        % -------------------------------------------------------------
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
        Initialized = false;

        % State: [px py pz vx vy vz]'
        StateX = nan(6, 1);
        StateP = nan(6, 6);
        LastStateTimestamp = NaN;

        % Debug counters
        NumImuSamples = 0;
        NumImuPredictions = 0;
        NumUwbCorrections = 0;
        NumFallbacks = 0;
        NumInvalidInputs = 0;
        NumOutputs = 0;

        LastStatus = 'not_started';
        LastFallbackReason = 'none';
        LastUsedImu = false;
        LastImuPredictionDt = NaN;

        % Logging
        LogFileId = -1;
        LogHeaderWritten = false;
    end

    % =====================================================================
    % PUBLIC METHODS
    % =====================================================================
    methods

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

            obj.resetStateOnly();
            obj.openLogIfNeeded();
        end

        function delete(obj)
            obj.closeLog();
        end

        function configure(obj, config)
            names = fieldnames(config);

            for i = 1:numel(names)
                name = names{i};

                if isprop(obj, name)
                    obj.(name) = config.(name);
                end
            end
        end

        function imu_out = processImuSample(obj, imu_sample)
            imu_out = obj.parseImuSample(imu_sample);

            if ~imu_out.valid
                obj.NumInvalidInputs = obj.NumInvalidInputs + 1;
                obj.LastStatus = 'invalid_imu_sample';
                return;
            end

            obj.LatestImuSample = imu_out;
            obj.HasImuSample = true;
            obj.NumImuSamples = obj.NumImuSamples + 1;
            obj.LastStatus = 'imu_sample_received';

            if obj.USE_IMU_FILTER && obj.USE_IMU_PREDICTION && obj.Initialized
                prediction_ok = obj.predictWithImuSample(imu_out);

                if prediction_ok
                    obj.LastStatus = 'imu_prediction_update';
                    obj.LastUsedImu = true;
                    obj.NumOutputs = obj.NumOutputs + 1;

                    obj.LastOutputStruct = obj.makeOutputFromCurrentState( ...
                        imu_out.timestamp, ...
                        'imu_prediction_update', ...
                        true, ...
                        false, ...
                        obj.LatestUwbGeneral, ...
                        imu_out);

                    obj.logFusedData(obj.LastOutputStruct);
                end
            end
        end

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
                    [NaN NaN NaN], ...
                    [NaN NaN NaN], ...
                    [NaN NaN NaN], ...
                    NaN, ...
                    false, ...
                    'invalid_uwb_general_input', ...
                    false, ...
                    true, ...
                    uwb, ...
                    obj.LatestImuSample);

                obj.LastOutputStruct = uwb_imu_filtered;
                obj.logFusedData(uwb_imu_filtered);
                return;
            end

            imu_available = obj.isImuAvailableForTimestamp(uwb.timestamp);

            % If IMU fusion is disabled or no fresh IMU is available, do not apply a
            % second Kalman layer. Just pass through the GeneralFilter output.
            if ~obj.USE_IMU_FILTER || ~imu_available
                if ~obj.USE_IMU_FILTER
                    status = 'passthrough_imu_filter_disabled';
                    obj.LastFallbackReason = 'imu_filter_disabled';
                else
                    status = 'passthrough_imu_missing';
                    obj.LastFallbackReason = 'imu_missing_or_old';
                end

                uwb_imu_filtered = obj.makeOutputStruct( ...
                    uwb.position, ...
                    uwb.velocity, ...
                    obj.getOrientationForOutput(false), ...
                    uwb.timestamp, ...
                    true, ...
                    status, ...
                    false, ...
                    true, ...
                    uwb, ...
                    obj.LatestImuSample);

                obj.LastOutputStruct = uwb_imu_filtered;
                obj.LastStatus = status;
                obj.NumFallbacks = obj.NumFallbacks + 1;
                obj.NumOutputs = obj.NumOutputs + 1;

                obj.logFusedData(uwb_imu_filtered);
                return;
            end

            % New structure:
            % The IMU filter only filters/fuses the measured sensor position.
            % Robot-centre compensation is handled later inside ControlLimo.m
            % or ControlBebop.m.
            corrected_measurement = uwb.position;
            corrected_velocity = uwb.velocity;

            if ~obj.Initialized
                obj.initializeFromUwb(corrected_measurement, corrected_velocity, uwb.timestamp);

                status = 'initialized_from_uwb';
                used_imu = false;
                fallback_used = false;

            elseif obj.USE_IMU_FILTER && imu_available
                obj.correctWithUwb(corrected_measurement, corrected_velocity, uwb.timestamp);

                status = 'uwb_imu_corrected';
                used_imu = true;
                fallback_used = false;
                obj.LastUsedImu = true;

            else
                % Keep the state synchronized with UWB even when no IMU is used.
                obj.correctWithUwb(corrected_measurement, corrected_velocity, uwb.timestamp);

                used_imu = false;

                if ~obj.USE_IMU_FILTER
                    status = 'uwb_only_correction';
                    fallback_used = false;
                    obj.LastFallbackReason = 'imu_filter_disabled';

                elseif ~imu_available
                    status = 'uwb_correction_imu_missing';
                    fallback_used = obj.FALLBACK_TO_GENERAL_FILTER;
                    obj.LastFallbackReason = 'imu_missing_or_old';
                    obj.NumFallbacks = obj.NumFallbacks + 1;

                else
                    status = 'uwb_correction';
                    fallback_used = false;
                end
            end

            orientation = obj.getOrientationForOutput(imu_available);

            % Fallback behaviour: if no valid state exists for any reason,
            % fall back to the GeneralFilter output.
            if ~obj.Initialized || any(~isfinite(obj.StateX))
                obj.NumFallbacks = obj.NumFallbacks + 1;
                obj.LastFallbackReason = 'state_not_initialized';

                position = corrected_measurement;
                velocity = corrected_velocity;
                status = 'fallback_general_output';
                fallback_used = true;
                used_imu = false;
            else
                position = obj.StateX(1:3).';
                velocity = obj.StateX(4:6).';
            end

            uwb_imu_filtered = obj.makeOutputStruct( ...
                position, ...
                velocity, ...
                orientation, ...
                uwb.timestamp, ...
                true, ...
                status, ...
                used_imu, ...
                fallback_used, ...
                uwb, ...
                obj.LatestImuSample);

            obj.LastOutputStruct = uwb_imu_filtered;
            obj.LastStatus = status;
            obj.NumOutputs = obj.NumOutputs + 1;

            obj.logFusedData(uwb_imu_filtered);
        end

        function output = getOutput(obj)
            if ~isempty(obj.LastOutputStruct)
                output = obj.LastOutputStruct;
            else
                output = obj.makeEmptyOutput();
            end
        end

        function imu_sample = getLatestImuSample(obj)
            if obj.HasImuSample
                imu_sample = obj.LatestImuSample;
            else
                imu_sample = obj.makeEmptyImuSample();
            end
        end

        function debug = getDebugInfo(obj)
            debug.num_imu_samples = obj.NumImuSamples;
            debug.num_imu_predictions = obj.NumImuPredictions;
            debug.num_uwb_corrections = obj.NumUwbCorrections;
            debug.num_fallbacks = obj.NumFallbacks;
            debug.num_invalid_inputs = obj.NumInvalidInputs;
            debug.num_outputs = obj.NumOutputs;

            debug.last_status = obj.LastStatus;
            debug.last_fallback_reason = obj.LastFallbackReason;
            debug.last_used_imu = obj.LastUsedImu;
            debug.last_imu_prediction_dt = obj.LastImuPredictionDt;

            debug.has_imu_sample = obj.HasImuSample;
            debug.has_uwb_sample = obj.HasUwbSample;
            debug.initialized = obj.Initialized;
            debug.state_timestamp = obj.LastStateTimestamp;
        end

        function reset(obj)
            obj.LatestImuSample = obj.makeEmptyImuSample();
            obj.LatestUwbGeneral = obj.makeEmptyUwbGeneralSample();
            obj.LastOutputStruct = obj.makeEmptyOutput();

            obj.HasImuSample = false;
            obj.HasUwbSample = false;

            obj.NumImuSamples = 0;
            obj.NumImuPredictions = 0;
            obj.NumUwbCorrections = 0;
            obj.NumFallbacks = 0;
            obj.NumInvalidInputs = 0;
            obj.NumOutputs = 0;

            obj.LastStatus = 'reset';
            obj.LastFallbackReason = 'none';
            obj.LastUsedImu = false;
            obj.LastImuPredictionDt = NaN;

            obj.resetStateOnly();
        end

        function setLogFile(obj, log_file_path)
            obj.closeLog();

            obj.LOG_FILE_PATH = char(string(log_file_path));
            obj.LogHeaderWritten = false;

            obj.openLogIfNeeded();
        end

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

        function resetStateOnly(obj)
            obj.Initialized = false;
            obj.StateX = nan(6, 1);
            obj.StateP = diag([1, 1, 1, 1, 1, 1]);
            obj.LastStateTimestamp = NaN;
        end

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

        function imu = parseImuSample(obj, input)
            imu = obj.makeEmptyImuSample();

            if ~isstruct(input)
                return;
            end

            if isfield(input, 'accel_body') && numel(input.accel_body) >= 3
                imu.accel_body = reshape(input.accel_body(1:3), 1, 3);
            elseif all(isfield(input, {'ax', 'ay', 'az'}))
                imu.accel_body = [input.ax, input.ay, input.az];
            end

            if isfield(input, 'gyro_body') && numel(input.gyro_body) >= 3
                imu.gyro_body = reshape(input.gyro_body(1:3), 1, 3);
            elseif all(isfield(input, {'gx', 'gy', 'gz'}))
                imu.gyro_body = [input.gx, input.gy, input.gz];
            end

            if isfield(input, 'orientation') && numel(input.orientation) >= 3
                imu.orientation = reshape(input.orientation(1:3), 1, 3);
            elseif all(isfield(input, {'roll', 'pitch', 'yaw'}))
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
                imu.valid = true;
            end

            has_some_data = any(isfinite(imu.accel_body)) || ...
                            any(isfinite(imu.gyro_body)) || ...
                            any(isfinite(imu.orientation));

            imu.valid = imu.valid && has_some_data && isfinite(imu.timestamp);
        end

        function available = isImuAvailableForTimestamp(obj, requested_timestamp)
            available = false;

            if ~obj.HasImuSample || ~obj.LatestImuSample.valid
                return;
            end

            if ~isfinite(requested_timestamp) || ~isfinite(obj.LatestImuSample.timestamp)
                return;
            end

            age = abs(requested_timestamp - obj.LatestImuSample.timestamp);
            available = age <= obj.IMU_FRESH_TIMEOUT;
        end

        function initializeFromUwb(obj, position, velocity, timestamp)
            x0 = zeros(6, 1);
            x0(1:3) = position(:);

            if obj.USE_UWB_VELOCITY_INIT && all(isfinite(velocity))
                x0(4:6) = velocity(:);
            end

            if strcmpi(obj.ROBOT_MODE, 'GROUND_2D')
                x0(3) = position(3);
                x0(6) = 0;
            end

            obj.StateX = x0;
            obj.StateP = diag([0.10, 0.10, 0.10, 1.00, 1.00, 1.00]);
            obj.LastStateTimestamp = timestamp;
            obj.Initialized = true;
        end

        function ok = predictWithImuSample(obj, imu)
            ok = false;

            if ~obj.Initialized || ~imu.valid || ~obj.USE_IMU_FILTER || ~obj.USE_IMU_PREDICTION
                return;
            end

            if ~isfinite(obj.LastStateTimestamp) || ~isfinite(imu.timestamp)
                return;
            end

            dt = imu.timestamp - obj.LastStateTimestamp;

            if ~isfinite(dt) || dt <= 0
                return;
            end

            dt = min(dt, obj.MAX_IMU_DT);
            obj.LastImuPredictionDt = dt;

            accel_world = obj.getWorldAcceleration(imu);

            F = obj.buildStateTransition(dt);
            B = obj.buildInputMatrix(dt);
            Q = obj.buildProcessNoise(dt);

            obj.StateX = F * obj.StateX + B * accel_world;
            obj.StateP = F * obj.StateP * F.' + Q;

            if strcmpi(obj.ROBOT_MODE, 'GROUND_2D')
                obj.StateX(6) = 0;
            end

            obj.LastStateTimestamp = imu.timestamp;
            obj.NumImuPredictions = obj.NumImuPredictions + 1;

            ok = true;
        end

        function accel_world = getWorldAcceleration(obj, imu)
            accel_body = imu.accel_body(:);
            accel_body = accel_body - obj.ACCEL_BIAS_BODY(:);

            if ~all(isfinite(accel_body))
                accel_world = [0; 0; 0];
                return;
            end

            orientation = imu.orientation(:).';

            if obj.USE_TILT_COMPENSATION && all(isfinite(orientation))
                R = obj.makeRotationMatrix(orientation);
                accel_world = R * accel_body;

            else
                if strcmpi(obj.ROBOT_MODE, 'GROUND_2D') && isfinite(orientation(3))
                    yaw = orientation(3);
                    R = obj.makeYawRotation(yaw);
                    accel_world = R * accel_body;
                else
                    accel_world = accel_body;
                end
            end

            if obj.REMOVE_GRAVITY
                accel_world = accel_world - [0; 0; obj.GRAVITY];
            end

            if strcmpi(obj.ROBOT_MODE, 'GROUND_2D')
                accel_world(3) = 0;
            end
        end

        function correctWithUwb(obj, measurement_position, measurement_velocity, timestamp)
            if ~obj.Initialized
                obj.initializeFromUwb(measurement_position, measurement_velocity, timestamp);
                return;
            end

            if isfinite(timestamp) && isfinite(obj.LastStateTimestamp) && timestamp > obj.LastStateTimestamp
                dt = timestamp - obj.LastStateTimestamp;
                dt = min(dt, obj.MAX_IMU_DT);

                F = obj.buildStateTransition(dt);
                Q = obj.buildProcessNoise(dt);

                obj.StateX = F * obj.StateX;
                obj.StateP = F * obj.StateP * F.' + Q;
                obj.LastStateTimestamp = timestamp;
            else
                obj.LastStateTimestamp = timestamp;
            end

            H = [eye(3), zeros(3)];
            R = diag(obj.MEASUREMENT_NOISE_POSITION(:).^2);

            z = measurement_position(:);
            y = z - H * obj.StateX;

            S = H * obj.StateP * H.' + R;
            K = obj.StateP * H.' / S;

            obj.StateX = obj.StateX + K * y;
            obj.StateP = (eye(6) - K * H) * obj.StateP;

            if all(isfinite(measurement_velocity))
                obj.StateX(4:6) = ...
                    (1 - obj.UWB_VELOCITY_BLEND) * obj.StateX(4:6) + ...
                    obj.UWB_VELOCITY_BLEND * measurement_velocity(:);
            end

            if strcmpi(obj.ROBOT_MODE, 'GROUND_2D')
                obj.StateX(6) = 0;
            end
        end

        function orientation = getOrientationForOutput(obj, imu_available)
            orientation = [NaN NaN NaN];

            if imu_available && obj.HasImuSample && obj.LatestImuSample.valid
                orientation = obj.LatestImuSample.orientation;
            elseif obj.HasImuSample && obj.LatestImuSample.valid
                orientation = obj.LatestImuSample.orientation;
            end
        end

        function F = buildStateTransition(~, dt)
            F = [1 0 0 dt 0 0; ...
                 0 1 0 0 dt 0; ...
                 0 0 1 0 0 dt; ...
                 0 0 0 1 0 0; ...
                 0 0 0 0 1 0; ...
                 0 0 0 0 0 1];
        end

        function B = buildInputMatrix(~, dt)
            B = [0.5 * dt^2 0 0; ...
                 0 0.5 * dt^2 0; ...
                 0 0 0.5 * dt^2; ...
                 dt 0 0; ...
                 0 dt 0; ...
                 0 0 dt];
        end

        function Q = buildProcessNoise(obj, dt)
            q_pos = obj.PROCESS_NOISE_POSITION.^2;
            q_vel = obj.PROCESS_NOISE_VELOCITY.^2;

            Q = diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel]);
            Q = Q * max(dt, 0.001);
        end

        function R = makeRotationMatrix(obj, orientation)
            if strcmpi(obj.ROBOT_MODE, 'GROUND_2D')
                yaw = orientation(3);
                R = obj.makeYawRotation(yaw);
                return;
            end

            roll = orientation(1);
            pitch = orientation(2);
            yaw = orientation(3);

            % ReadBebop.m and ReadLimo.m should output roll/pitch/yaw in degrees.
            cr = cosd(roll);  sr = sind(roll);
            cp = cosd(pitch); sp = sind(pitch);
            cy = cosd(yaw);   sy = sind(yaw);

            Rx = [1 0 0; ...
                  0 cr -sr; ...
                  0 sr cr];

            Ry = [cp 0 sp; ...
                  0 1 0; ...
                  -sp 0 cp];

            Rz = [cy -sy 0; ...
                  sy cy 0; ...
                  0 0 1];

            R = Rz * Ry * Rx;
        end

        function R = makeYawRotation(~, yaw)
            % ReadBebop.m and ReadLimo.m should output yaw in degrees.
            cy = cosd(yaw);
            sy = sind(yaw);

            R = [cy -sy 0; ...
                 sy cy 0; ...
                 0 0 1];
        end

        function output = makeOutputFromCurrentState(obj, timestamp, status, used_imu, fallback_used, uwb, imu)
            if ~obj.Initialized || any(~isfinite(obj.StateX))
                output = obj.makeEmptyOutput();
                output.status = status;
                return;
            end

            output = obj.makeOutputStruct( ...
                obj.StateX(1:3).', ...
                obj.StateX(4:6).', ...
                obj.getOrientationForOutput(true), ...
                timestamp, ...
                true, ...
                status, ...
                used_imu, ...
                fallback_used, ...
                uwb, ...
                imu);
        end

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

        function imu = makeEmptyImuSample(~)
            imu.accel_body = [NaN NaN NaN];
            imu.gyro_body = [NaN NaN NaN];
            imu.orientation = [NaN NaN NaN];
            imu.timestamp = NaN;
            imu.valid = false;
            imu.source = 'none';
        end

        function output = makeEmptyOutput(obj)
            uwb = obj.makeEmptyUwbGeneralSample();
            imu = obj.makeEmptyImuSample();

            output = obj.makeOutputStruct( ...
                [NaN NaN NaN], ...
                [NaN NaN NaN], ...
                [NaN NaN NaN], ...
                NaN, ...
                false, ...
                'not_initialized', ...
                false, ...
                true, ...
                uwb, ...
                imu);
        end

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

        function logFusedData(obj, output)
            if ~obj.LOG_TO_CSV || obj.LogFileId <= 0
                return;
            end

            if ~obj.LogHeaderWritten
                fprintf(obj.LogFileId, [ ...
                    'timestamp,status,position_x,position_y,position_z,', ...
                    'velocity_x,velocity_y,velocity_z,roll,pitch,yaw,quality,valid,', ...
                    'used_imu,fallback_used,imu_valid,imu_source,input_uwb_source,', ...
                    'input_uwb_valid,input_uwb_accepted,input_uwb_rejection_reason,', ...
                    'accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,', ...
                    'num_predictions,last_prediction_dt,last_fallback_reason\n']);

                obj.LogHeaderWritten = true;
            end

            fprintf(obj.LogFileId, [ ...
                '%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,', ...
                '%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%s,%s,%d,%d,%s,', ...
                '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%s\n'], ...
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
                obj.NumImuPredictions, ...
                obj.LastImuPredictionDt, ...
                obj.safeCsvText(obj.LastFallbackReason));
        end

        function t = getCurrentTimestamp(~)
            try
                t = posixtime(datetime('now'));
            catch
                t = now * 24 * 3600; %#ok<TNOW1>
            end
        end

        function txt = safeCsvText(~, value)
            txt = char(string(value));
            txt = strrep(txt, ',', '_');
            txt = strrep(txt, newline, ' ');
        end
    end
end
