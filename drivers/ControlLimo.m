% =========================================================================
% CONTROL LIMO
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
% Limo / wheeled robot control wrapper for the MATLAB localization structure.
%
% New input format:
%   update(final_position, final_angles)
%
%   final_position.position  = [x y z]
%   final_position.velocity  = [vx vy vz] optional
%   final_position.timestamp = time optional
%   final_position.valid     = true/false optional
%   final_position.source    = string optional
%
%   final_angles.roll        = roll  [deg]
%   final_angles.pitch       = pitch [deg]
%   final_angles.yaw         = yaw   [deg]
%   final_angles.timestamp   = time optional
%   final_angles.valid       = true/false optional
%   final_angles.source      = string optional
%
% Design note:
% This controller receives the already-selected final_position and
% final_angles from MatlabMasterUWBControl.m.
%
% =========================================================================

classdef ControlLimo < handle

    properties
        %% ROS / robot objects
        RobotObj
        pub
        msg

        %% Optional internal IMU fallback
        % ReadLimo.m should normally provide final_angles.
        % This subscriber is kept only as a fallback so the old working logic
        % is not broken if final_angles is missing.
        USE_INTERNAL_IMU_FALLBACK = true;
        imu_sub
        latest_accel = [0, 0, 0];
        latest_yaw = 0;      % [deg]
        latest_roll = 0;     % [deg]
        latest_pitch = 0;    % [deg]

        %% Control position offset
        % This offset converts the incoming measured/selected position to the
        % position that should be used for Limo control.
        %
        % Definition:
        %   robot_control_position = incoming_position + CONTROL_POSITION_OFFSET_WORLD
        %
        % For the Limo this is kept as a simple world-frame offset. It is
        % mainly used to move the measured point down to the robot centre.
        APPLY_CONTROL_POSITION_OFFSET = true;
        CONTROL_POSITION_OFFSET_WORLD = [0; -0.006; -0.18];  % [m]

        %% Legacy compatibility
        % Old offsets from the previous controller, kept for reference.
        % Only used in legacy update mode.
        legacy_x_offset = 0;
        legacy_y_offset = -0.006;
        legacy_z_offset = -0.18;

        % Setting kept for backwards compatibility.
        % The master script normally chooses final_position.
        use_uwb_for_control = false;

        %% Trajectory configuration
        trajectory_mode = 'infinity';   % options: 'infinity', 'setpoints', 'circle'
        rX = 1;                         % X radius movement [m]
        rY = 1;                         % Y radius movement [m]
        T = 15;                         % time per movement [s]
        Kp = 0.5;                       % proportional gain
        a = 0.15;                       % feedback linearization point offset

        %% State
        t_start
        w
        nLandMsg = 5;
        EmergencyTriggered = false;
        last_final_position
        last_final_angles
    end

    methods
        function obj = ControlLimo()
            fprintf('[LIMO CONTROL] Initializing Limo controller...\n');

            try
                obj.pub = rospublisher('/L1/cmd_vel', 'geometry_msgs/Twist');
                obj.msg = rosmessage(obj.pub);

                if obj.USE_INTERNAL_IMU_FALLBACK
                    obj.imu_sub = rossubscriber('/L1/imu', 'sensor_msgs/Imu', @obj.imuCallback);
                end

                % Same robot object as the previous controller.
                obj.RobotObj = RPioneer(1, 'RosAria', 1);
                obj.w = 2*pi/obj.T;

                fprintf('[LIMO CONTROL] Ready.\n');

            catch ME
                fprintf('[LIMO CONTROL] Error during initialization:\n%s\n', ME.message);
                fprintf('[LIMO CONTROL] Controller object created, but robot commands may not work.\n');
            end
        end

        function perform_movement(obj)
            fprintf('[LIMO CONTROL] Starting movement trajectory...\n');
            obj.t_start = tic;
        end

        function update(obj, final_position, final_angles)
            % UPDATE New interface:
            %   update(final_position, final_angles)
            %
            % Legacy interface is also supported:
            %   update(uwb_pos, opti_pos)
            % In legacy mode, use_uwb_for_control selects the vector.

            if nargin < 3
                final_angles = obj.makeInvalidAngles();
            end

            if isempty(obj.t_start)
                obj.t_start = tic;
            end

            t_atual = toc(obj.t_start);

            % -------------------------------------------------------------
            % 1. Read position and angles from new or legacy format
            % -------------------------------------------------------------
            legacy_mode = isnumeric(final_position) && isnumeric(final_angles);

            if legacy_mode
                uwb_pos = final_position(:);
                opti_pos = final_angles(:);

                if obj.use_uwb_for_control
                    current_pos = obj.padVector3(uwb_pos);
                else
                    current_pos = obj.padVector3(opti_pos);
                end

                % Preserve legacy offset behaviour for legacy calls.
                current_pos = current_pos + [obj.legacy_x_offset; obj.legacy_y_offset; obj.legacy_z_offset];

                angles = obj.makeInvalidAngles();
                angles.yaw = obj.latest_yaw;
                angles.valid = true;

            else
                [current_pos, pos_valid] = obj.extractPosition(final_position);
                angles = obj.extractAngles(final_angles);

                if ~pos_valid
                    fprintf('[LIMO CONTROL] Invalid final_position. Sending stop command.\n');
                    obj.stop();
                    return;
                end

                current_pos = obj.applyControlPositionOffset(current_pos);
            end

            obj.last_final_position = final_position;
            obj.last_final_angles = angles;

            % -------------------------------------------------------------
            % 2. Select yaw
            % -------------------------------------------------------------
            if isfield(angles, 'valid') && angles.valid && ...
                    isfield(angles, 'yaw') && isfinite(angles.yaw)
                psi = angles.yaw;   % [deg]
            else
                % Fallback to internal IMU callback from old controller.
                psi = obj.latest_yaw;  % [deg]
            end

            % -------------------------------------------------------------
            % 3. Update robot state
            % -------------------------------------------------------------
            if isempty(obj.RobotObj)
                fprintf('[LIMO CONTROL] RobotObj is empty. Cannot send command.\n');
                return;
            end

            if length(obj.RobotObj.pPos.X) < 12
                obj.RobotObj.pPos.X = zeros(12,1);
            end

            obj.RobotObj.pPos.X(1:3) = current_pos(:);
            obj.RobotObj.pPos.X(4) = angles.roll;    % [deg]
            obj.RobotObj.pPos.X(5) = angles.pitch;   % [deg]
            obj.RobotObj.pPos.X(6) = psi;            % [deg]

            % -------------------------------------------------------------
            % 4. Original trajectory generation logic
            % -------------------------------------------------------------
            switch obj.trajectory_mode
                case 'circle'
                    obj.RobotObj.pPos.Xd(1:2) = [obj.rX*cos(obj.w*t_atual); ...
                                                  obj.rY*sin(obj.w*t_atual)];
                    obj.RobotObj.pPos.Xd(7:8) = [-obj.rX*obj.w*sin(obj.w*t_atual); ...
                                                   obj.rY*obj.w*cos(obj.w*t_atual)];

                case 'setpoints'
                    setPointsX = [1, -1, -1, 1];
                    setPointsY = [1, 1, -1, -1];
                    ptIndex = mod(floor(t_atual / 5), 4) + 1;

                    obj.RobotObj.pPos.Xd(1:2) = [setPointsX(ptIndex); setPointsY(ptIndex)];
                    obj.RobotObj.pPos.Xd(7:8) = [0; 0];

                case 'infinity'
                    obj.RobotObj.pPos.Xd(1:2) = [obj.rX*sin(obj.w*t_atual); ...
                                                  obj.rY*sin(2*obj.w*t_atual)];
                    obj.RobotObj.pPos.Xd(7:8) = [obj.rX*obj.w*cos(obj.w*t_atual); ...
                                                  2*obj.rY*obj.w*cos(2*obj.w*t_atual)];

                otherwise
                    fprintf('[LIMO CONTROL] Unknown trajectory_mode: %s. Stopping.\n', obj.trajectory_mode);
                    obj.stop();
                    return;
            end

            % -------------------------------------------------------------
            % 5. Original feedback-linearization control logic
            % -------------------------------------------------------------
            % final_angles are in degrees, so use cosd/sind here.
            K = [cosd(psi), -obj.a*sind(psi); ...
                 sind(psi),  obj.a*cosd(psi)];

            dXd = obj.RobotObj.pPos.Xd(7:8);
            Xd = obj.RobotObj.pPos.Xd(1:2);
            X = obj.RobotObj.pPos.X(1:2);

            u = K \ (dXd + obj.Kp*(Xd - X));

            % -------------------------------------------------------------
            % 6. Send control signals
            % -------------------------------------------------------------
            obj.msg.Linear.X = u(1);
            obj.msg.Angular.Z = u(2);
            send(obj.pub, obj.msg);
        end

        function stop(obj)
            if isempty(obj.pub) || isempty(obj.msg)
                return;
            end

            obj.msg.Linear.X = 0;
            obj.msg.Linear.Y = 0;
            obj.msg.Linear.Z = 0;
            obj.msg.Angular.X = 0;
            obj.msg.Angular.Y = 0;
            obj.msg.Angular.Z = 0;
            send(obj.pub, obj.msg);
        end

        function landAndStop(obj)
            fprintf('[LIMO CONTROL] Initiating stop sequence...\n');

            for i = 1:obj.nLandMsg
                obj.stop();
                pause(0.1);
            end
        end

        function imuCallback(obj, ~, message)
            % Fallback IMU reader.
            % The preferred new path is ReadLimo.m.
            % Angles are stored in degrees.

            obj.latest_accel = [message.LinearAcceleration.X, ...
                                message.LinearAcceleration.Y, ...
                                message.LinearAcceleration.Z];

            qW = message.Orientation.W;
            qX = message.Orientation.X;
            qY = message.Orientation.Y;
            qZ = message.Orientation.Z;

            [roll, pitch, yaw] = obj.quatToRpy(qW, qX, qY, qZ);

            obj.latest_roll = roll;
            obj.latest_pitch = pitch;
            obj.latest_yaw = yaw;
        end
    end

    methods (Access = private)
        function position = applyControlPositionOffset(obj, position)
            if obj.APPLY_CONTROL_POSITION_OFFSET
                position = position(:) + obj.CONTROL_POSITION_OFFSET_WORLD(:);
            else
                position = position(:);
            end
        end

        function [position, valid] = extractPosition(obj, final_position)
            valid = false;
            position = [NaN; NaN; NaN];

            if isstruct(final_position)
                if isfield(final_position, 'position')
                    position = obj.padVector3(final_position.position);
                end

                if isfield(final_position, 'valid')
                    valid = logical(final_position.valid);
                else
                    valid = all(isfinite(position));
                end

            elseif isnumeric(final_position)
                position = obj.padVector3(final_position);
                valid = all(isfinite(position));
            end

            if ~all(isfinite(position))
                valid = false;
            end
        end

        function angles = extractAngles(obj, final_angles)
            angles = obj.makeInvalidAngles();

            if isstruct(final_angles)
                if isfield(final_angles, 'roll');      angles.roll = final_angles.roll; end
                if isfield(final_angles, 'pitch');     angles.pitch = final_angles.pitch; end
                if isfield(final_angles, 'yaw');       angles.yaw = final_angles.yaw; end
                if isfield(final_angles, 'timestamp'); angles.timestamp = final_angles.timestamp; end
                if isfield(final_angles, 'source');    angles.source = string(final_angles.source); end

                if isfield(final_angles, 'valid')
                    angles.valid = logical(final_angles.valid);
                else
                    angles.valid = all(isfinite([angles.roll, angles.pitch, angles.yaw]));
                end
            end

            % If missing, use internal fallback values.
            if ~angles.valid
                angles.roll = obj.latest_roll;
                angles.pitch = obj.latest_pitch;
                angles.yaw = obj.latest_yaw;
                angles.valid = true;
                angles.source = "limo_control_internal_fallback";
            end
        end

        function angles = makeInvalidAngles(~)
            angles = struct();
            angles.roll = 0;       % [deg]
            angles.pitch = 0;      % [deg]
            angles.yaw = 0;        % [deg]
            angles.timestamp = NaN;
            angles.valid = false;
            angles.source = "none";
        end

        function v = padVector3(~, input_vector)
            v = input_vector(:);

            if numel(v) < 3
                v(end+1:3,1) = 0;
            elseif numel(v) > 3
                v = v(1:3);
            end
        end

        function [roll, pitch, yaw] = quatToRpy(~, qW, qX, qY, qZ)
            % Quaternion to roll, pitch, yaw in degrees.

            sinr_cosp = 2*(qW*qX + qY*qZ);
            cosr_cosp = 1 - 2*(qX^2 + qY^2);
            roll = atan2d(sinr_cosp, cosr_cosp);

            sinp = 2*(qW*qY - qZ*qX);
            if abs(sinp) >= 1
                pitch = sign(sinp) * 90;
            else
                pitch = asind(sinp);
            end

            siny_cosp = 2*(qW*qZ + qX*qY);
            cosy_cosp = 1 - 2*(qY^2 + qZ^2);
            yaw = atan2d(siny_cosp, cosy_cosp);
        end
    end
end
