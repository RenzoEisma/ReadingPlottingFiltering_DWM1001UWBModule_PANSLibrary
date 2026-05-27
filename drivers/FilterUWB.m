% =========================================================================
% MATLAB UWB KALMAN FILTER
% Author: Renzo Eisma
% Date: 09/04/2026
% Description: 3D Constant Velocity Kalman Filter with Outlier Rejection
% =========================================================================
classdef FilterUWB < handle
    properties
        IsInitialized = false;
        
        % Filter State: [x; y; z; vx; vy; vz]
        X; 
        
        % State Covariance Matrix (Uncertainty)
        P; 
        
        % Time step (Set this to the actual UWB update rate, e.g., 0.1 for 10Hz)
        dt = 0.1; 
        
        % Maximum allowed speed (m/s) for outlier rejection
        MaxSpeed = 15.0; 
        
        % Matrices
        F; % State Transition
        H; % Measurement Mapping
        Q; % Process Noise (Trust in physics model)
        R; % Measurement Noise (Trust in UWB sensor)
    end
    
    methods
        function obj = FilterUWB(dt_val)
            if nargin > 0
                obj.dt = dt_val;
            end
            
            % Initialize Matrices
            % State transition: Position = old_pos + velocity * dt
            obj.F = [1 0 0 obj.dt 0 0;
                     0 1 0 0 obj.dt 0;
                     0 0 1 0 0 obj.dt;
                     0 0 0 1 0 0;
                     0 0 0 0 1 0;
                     0 0 0 0 0 1];
                 
            % Measurement matrix: We only measure [x, y, z], not velocity
            obj.H = [1 0 0 0 0 0;
                     0 1 0 0 0 0;
                     0 0 1 0 0 0];
                 
            % Tune these values based on setup
            % Higher Q = more responsive (less lag), but more noise
            obj.Q = eye(6) * 0.5; 
            
            % Higher R = smoother, but more lag (trusts measurement less)
            obj.R = eye(3) * 2.0; 
        end
        
        function [filt_x, filt_y, filt_z] = process(obj, raw_x, raw_y, raw_z)
            Z = [raw_x; raw_y; raw_z];
            
            % 1. INITIALIZATION
            if ~obj.IsInitialized
                obj.X = [Z; 0; 0; 0]; % Start with current pos, 0 velocity
                obj.P = eye(6) * 10;  % High initial uncertainty
                obj.IsInitialized = true;
                filt_x = Z(1); filt_y = Z(2); filt_z = Z(3);
                return;
            end
            
            % 2. PREDICT STAGE
            % Predict where the drone is based on its last known velocity
            X_pred = obj.F * obj.X;
            P_pred = obj.F * obj.P * obj.F' + obj.Q;
            
            % 3. OUTLIER REJECTION (Kinematic Gate)
            % Calculate distance from prediction to new measurement
            predicted_pos = X_pred(1:3);
            dist = norm(Z - predicted_pos);
            
            % If the implied speed (dist/dt) is impossible, ignore the measurement
            if (dist / obj.dt) > obj.MaxSpeed
                % Outlier detected! Rely purely on the prediction.
                obj.X = X_pred;
                obj.P = P_pred;
                
                filt_x = obj.X(1); filt_y = obj.X(2); filt_z = obj.X(3);
                return; 
            end
            
            % 4. UPDATE STAGE
            % Calculate Kalman Gain
            S = obj.H * P_pred * obj.H' + obj.R;
            K = P_pred * obj.H' / S;
            
            % Update State with measurement
            y = Z - (obj.H * X_pred); % Measurement residual
            obj.X = X_pred + (K * y);
            
            % Update Covariance
            I = eye(6);
            obj.P = (I - K * obj.H) * P_pred;
            
            % 5. OUTPUT
            filt_x = obj.X(1);
            filt_y = obj.X(2);
            filt_z = obj.X(3);
        end
        
        function reset(obj)
            obj.IsInitialized = false;
        end
    end
end