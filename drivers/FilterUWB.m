% =========================================================================
% MATLAB UWB FILTER
% Author: Renzo Eisma
% Date: 05/2026
% Description:
%   Tunable UWB position filter for live robot/drone localization.
%
%   Current filter options:
%   - Constant velocity Kalman filter
%   - Outlier rejection:
%       - maximum speed gate
%       - maximum position jump gate
%       - Mahalanobis innovation gate
%   - Optional low-pass output smoothing
%
%   EKF is intentionally not included here because this script receives
%   already calculated UWB x,y,z positions. EKF should be used later for:
%   - raw anchor distance triangulation
%   - UWB + IMU sensor fusion
% =========================================================================


classdef FilterUWB < handle

    % =====================================================================
    % USER TUNABLE FILTER SETTINGS
    % =====================================================================
    properties
        % -------------------------------------------------------------
        % Main filter switches
        % -------------------------------------------------------------
        USE_KALMAN_FILTER = true;
        USE_OUTLIER_REJECTION = true;
        USE_SPEED_GATE = true;
        USE_POSITION_JUMP_GATE = false;
        USE_MAHALANOBIS_GATE = true;
        USE_LOW_PASS_OUTPUT = false;

        % -------------------------------------------------------------
        % Timing
        % -------------------------------------------------------------
        dt = 0.1;                         % UWB update time [s], 0.1 = 10 Hz

        % -------------------------------------------------------------
        % Kalman tuning
        % Higher process noise = faster reaction, less smoothing
        % Higher measurement noise = more smoothing, more lag
        % -------------------------------------------------------------
        PROCESS_NOISE_POSITION = 0.05;
        PROCESS_NOISE_VELOCITY = 1.0;

        MEASUREMENT_NOISE_X = 0.5^2;     % expected UWB X noise [m^2]
        MEASUREMENT_NOISE_Y = 0.5^2;     % expected UWB Y noise [m^2]
        MEASUREMENT_NOISE_Z = 0.7^2;     % expected UWB Z noise [m^2]

        INITIAL_POSITION_UNCERTAINTY = 1.0;
        INITIAL_VELOCITY_UNCERTAINTY = 5.0;

        % -------------------------------------------------------------
        % Outlier rejection tuning
        % -------------------------------------------------------------
        MAX_ALLOWED_SPEED = 4.0;          % maximum believable movement speed [m/s]
        MAX_POSITION_JUMP = 1.0;          % maximum allowed jump between raw samples [m]
        MAHALANOBIS_GATE = 11.34;         % 99% gate for 3D measurement

        % -------------------------------------------------------------
        % Optional low-pass output smoothing
        % Alpha close to 1 = fast response
        % Alpha close to 0 = heavy smoothing / more delay
        % -------------------------------------------------------------
        LOW_PASS_ALPHA = 0.85;
    end


    % =====================================================================
    % INTERNAL FILTER VARIABLES
    % =====================================================================
    properties
        IsInitialized = false;

        % Filter state: [x; y; z; vx; vy; vz]
        X;
        P;

        % Matrices
        F;
        H;
        Q;
        R;

        % Previous values
        LastRawMeasurement = [];
        LastOutput = [];

        % Debug counters
        OutlierCount = 0;
        AcceptedCount = 0;
    end


    % =====================================================================
    % METHODS
    % =====================================================================
    methods

        % =================================================================
        % Constructor
        % =================================================================
        function obj = FilterUWB(dt_val)
            if nargin > 0
                obj.dt = dt_val;
            end

            obj.updateMatrices();
        end


        % =================================================================
        % Main process function
        % =================================================================
        function [filt_x, filt_y, filt_z] = process(obj, raw_x, raw_y, raw_z)

            Z = [raw_x; raw_y; raw_z];

            % -------------------------------------------------------------
            % Invalid input protection
            % -------------------------------------------------------------
            if any(isnan(Z)) || any(isinf(Z))
                [filt_x, filt_y, filt_z] = obj.outputPredictionOnly();
                return;
            end

            % -------------------------------------------------------------
            % First measurement initializes the filter
            % -------------------------------------------------------------
            if ~obj.IsInitialized
                obj.initializeFilter(Z);

                filt_x = Z(1);
                filt_y = Z(2);
                filt_z = Z(3);
                return;
            end

            % -------------------------------------------------------------
            % If Kalman is disabled, optionally only apply basic outlier
            % rejection and low-pass smoothing.
            % -------------------------------------------------------------
            if ~obj.USE_KALMAN_FILTER
                accepted = obj.simpleRawGate(Z);

                if accepted
                    output_pos = Z;
                    obj.LastRawMeasurement = Z;
                    obj.AcceptedCount = obj.AcceptedCount + 1;
                else
                    output_pos = obj.LastOutput;
                    obj.OutlierCount = obj.OutlierCount + 1;
                end

                output_pos = obj.applyLowPass(output_pos);

                filt_x = output_pos(1);
                filt_y = output_pos(2);
                filt_z = output_pos(3);
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
                accepted = obj.checkOutlierGates(Z, X_pred, P_pred);
            end

            % -------------------------------------------------------------
            % Kalman update or prediction-only output
            % -------------------------------------------------------------
            if accepted
                S = obj.H * P_pred * obj.H' + obj.R;
                K = P_pred * obj.H' / S;

                residual = Z - (obj.H * X_pred);

                obj.X = X_pred + (K * residual);
                obj.P = (eye(6) - K * obj.H) * P_pred;

                obj.LastRawMeasurement = Z;
                obj.AcceptedCount = obj.AcceptedCount + 1;
            else
                obj.X = X_pred;
                obj.P = P_pred;
                obj.OutlierCount = obj.OutlierCount + 1;
            end

            % -------------------------------------------------------------
            % Output
            % -------------------------------------------------------------
            output_pos = obj.X(1:3);
            output_pos = obj.applyLowPass(output_pos);

            filt_x = output_pos(1);
            filt_y = output_pos(2);
            filt_z = output_pos(3);
        end


        % =================================================================
        % Reset filter
        % =================================================================
        function reset(obj)
            obj.IsInitialized = false;
            obj.X = [];
            obj.P = [];
            obj.LastRawMeasurement = [];
            obj.LastOutput = [];
            obj.OutlierCount = 0;
            obj.AcceptedCount = 0;
            obj.updateMatrices();
        end


        % =================================================================
        % Update matrices from current tuning values
        % =================================================================
        function updateMatrices(obj)

            dt_val = obj.dt;

            % State transition matrix
            obj.F = [
                1 0 0 dt_val 0      0;
                0 1 0 0      dt_val 0;
                0 0 1 0      0      dt_val;
                0 0 0 1      0      0;
                0 0 0 0      1      0;
                0 0 0 0      0      1
            ];

            % Measurement matrix
            obj.H = [
                1 0 0 0 0 0;
                0 1 0 0 0 0;
                0 0 1 0 0 0
            ];

            % Process noise
            obj.Q = diag([
                obj.PROCESS_NOISE_POSITION;
                obj.PROCESS_NOISE_POSITION;
                obj.PROCESS_NOISE_POSITION;
                obj.PROCESS_NOISE_VELOCITY;
                obj.PROCESS_NOISE_VELOCITY;
                obj.PROCESS_NOISE_VELOCITY
            ]);

            % Measurement noise
            obj.R = diag([
                obj.MEASUREMENT_NOISE_X;
                obj.MEASUREMENT_NOISE_Y;
                obj.MEASUREMENT_NOISE_Z
            ]);
        end


        % =================================================================
        % Initialize filter state
        % =================================================================
        function initializeFilter(obj, Z)
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
            obj.LastOutput = Z;
            obj.IsInitialized = true;
        end


        % =================================================================
        % Outlier rejection gates
        % =================================================================
        function accepted = checkOutlierGates(obj, Z, X_pred, P_pred)

            accepted = true;
            predicted_pos = X_pred(1:3);

            % -------------------------------------------------------------
            % Speed gate
            % -------------------------------------------------------------
            if obj.USE_SPEED_GATE
                movement = norm(Z - predicted_pos);
                implied_speed = movement / obj.dt;

                if implied_speed > obj.MAX_ALLOWED_SPEED
                    accepted = false;
                    return;
                end
            end

            % -------------------------------------------------------------
            % Position jump gate
            % -------------------------------------------------------------
            if obj.USE_POSITION_JUMP_GATE && ~isempty(obj.LastRawMeasurement)
                jump = norm(Z - obj.LastRawMeasurement);

                if jump > obj.MAX_POSITION_JUMP
                    accepted = false;
                    return;
                end
            end

            % -------------------------------------------------------------
            % Mahalanobis innovation gate
            % -------------------------------------------------------------
            if obj.USE_MAHALANOBIS_GATE
                residual = Z - (obj.H * X_pred);
                S = obj.H * P_pred * obj.H' + obj.R;

                mahalanobis_distance = residual' / S * residual;

                if mahalanobis_distance > obj.MAHALANOBIS_GATE
                    accepted = false;
                    return;
                end
            end
        end


        % =================================================================
        % Simple raw gate for when Kalman is disabled
        % =================================================================
        function accepted = simpleRawGate(obj, Z)

            accepted = true;

            if ~obj.USE_OUTLIER_REJECTION || isempty(obj.LastRawMeasurement)
                return;
            end

            if obj.USE_SPEED_GATE
                movement = norm(Z - obj.LastRawMeasurement);
                implied_speed = movement / obj.dt;

                if implied_speed > obj.MAX_ALLOWED_SPEED
                    accepted = false;
                    return;
                end
            end

            if obj.USE_POSITION_JUMP_GATE
                jump = norm(Z - obj.LastRawMeasurement);

                if jump > obj.MAX_POSITION_JUMP
                    accepted = false;
                    return;
                end
            end
        end


        % =================================================================
        % Optional low-pass output smoothing
        % =================================================================
        function output_pos = applyLowPass(obj, input_pos)

            if ~obj.USE_LOW_PASS_OUTPUT || isempty(obj.LastOutput)
                output_pos = input_pos;
                obj.LastOutput = output_pos;
                return;
            end

            alpha = obj.LOW_PASS_ALPHA;

            output_pos = alpha * input_pos + (1 - alpha) * obj.LastOutput;
            obj.LastOutput = output_pos;
        end


        % =================================================================
        % Prediction-only output for invalid inputs before initialization
        % =================================================================
        function [filt_x, filt_y, filt_z] = outputPredictionOnly(obj)

            if obj.IsInitialized && obj.USE_KALMAN_FILTER
                obj.X = obj.F * obj.X;
                obj.P = obj.F * obj.P * obj.F' + obj.Q;

                output_pos = obj.applyLowPass(obj.X(1:3));
            elseif ~isempty(obj.LastOutput)
                output_pos = obj.LastOutput;
            else
                output_pos = [NaN; NaN; NaN];
            end

            filt_x = output_pos(1);
            filt_y = output_pos(2);
            filt_z = output_pos(3);
        end
    end
end