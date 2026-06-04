% =========================================================================
% CONTROL BEBOP
% Author: Renzo Eisma
% Assistance note:
%   ChatGPT Pro 5.5 Thinking Extended was used to clean up variable names,
%   comments and line spacing.
%   The original concept, code logic, and project structure were created by
%   a human author
% Date: 06/2026
%
% Purpose:
%   Bebop drone control wrapper for the new MATLAB localization structure.
%
% New input format:
%   update(final_position, final_angles)
%
%   final_position.position  = [x y z]
%   final_position.velocity  = [vx vy vz]       optional
%   final_position.timestamp = time             optional
%   final_position.valid     = true/false       optional
%   final_position.source    = string           optional
%
%   final_angles.roll        = roll  [rad]
%   final_angles.pitch       = pitch [rad]
%   final_angles.yaw         = yaw   [rad]
%   final_angles.timestamp   = time             optional
%   final_angles.valid       = true/false       optional
%   final_angles.source      = string           optional
%
% Design note:
%   This keeps the old Bebop control logic as much as possible. The main
%   change is that the selected final_position and final_angles now come from
%   the master script instead of this controller choosing UWB vs OptiTrack.
%
%   Offset compensation from UWB tag position to drone centre should normally
%   happen before this controller, inside ImuFusionFilter.m.
% =========================================================================

classdef ControlBebop < handle
    properties
        %% Robot object
        RobotObj

        %% Compatibility / offsets
        % final_position is expected to already be corrected to the
        % drone centre. Keep this false to avoid double offset compensation.
        APPLY_FINAL_POSITION_OFFSET = false;
        final_position_offset_world = [0; 0; 0];

        % Old offsets from previous controller, kept for reference.
        % Only used in legacy update mode or if enabled manually.
        legacy_x_offset = -0.03;
        legacy_y_offset = 0.009;
        legacy_z_offset = -0.135;

        % Setting kept for backwards compatibility.
        % The master script normally chooses final_position.
        use_uwb_for_control = false;

        %% Trajectory configuration
        trajectory_mode = 'takeoff_land_test';  % takeoff_land_test, infinity, setpoints, circle
        rX = 1;                                  % X radius movement [m]
        rY = 1;                                  % Y radius movement [m]
        hover_Z = 1;                             % target altitude [m]
        T = 15;                                  % time per movement [s]
        Kp = 0.8;                                % proportional gain

        %% State
        t_start
        w
        nLandMsg = 5;
        EmergencyTriggered = false;
        last_final_position
        last_final_angles
    end

    methods
        function obj = ControlBebop(namespace)
            if nargin < 1 || strlength(string(namespace)) == 0
                namespace = 'B1';
            end

            fprintf('[BEBOP CONTROL] Initializing Bebop controller...\n');

            try
                obj.RobotObj = Bebop(1, char(namespace));
                obj.RobotObj.pPos.X = zeros(12,1);
                obj.w = 2*pi/obj.T;

                fprintf('[BEBOP CONTROL] Ready. Namespace: %s\n', char(namespace));
            catch ME
                fprintf('[BEBOP CONTROL] Error during initialization:\n%s\n', ME.message);
                fprintf('[BEBOP CONTROL] Controller object created, but drone commands may not work.\n');
            end
        end

        function perform_movement(obj)
            if isempty(obj.RobotObj)
                fprintf('[BEBOP CONTROL] RobotObj is empty. Cannot take off.\n');
                return;
            end

            fprintf('[BEBOP CONTROL] Taking off...\n');
            obj.RobotObj.rTakeOff();

            % Preserve the previous takeoff wait so the drone can reach initial height.
            pause(4);

            fprintf('[BEBOP CONTROL] Starting movement trajectory...\n');
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
            else
                [current_pos, pos_valid] = obj.extractPosition(final_position);
                angles = obj.extractAngles(final_angles);

                if obj.APPLY_FINAL_POSITION_OFFSET
                    current_pos = current_pos + obj.final_position_offset_world;
                end

                if ~pos_valid
                    fprintf('[BEBOP CONTROL] Invalid final_position. Sending hover command.\n');
                    obj.hover();
                    return;
                end
            end

            obj.last_final_position = final_position;
            obj.last_final_angles = angles;

            % -------------------------------------------------------------
            % 2. Update drone state
            % -------------------------------------------------------------
            if isempty(obj.RobotObj)
                fprintf('[BEBOP CONTROL] RobotObj is empty. Cannot send command.\n');
                return;
            end

            if length(obj.RobotObj.pPos.X) < 12
                obj.RobotObj.pPos.X = zeros(12,1);
            end

            obj.RobotObj.pPos.X(1:3) = current_pos(:);

            % Store orientation when available. The proportional position
            % controller does not depend on these angles directly, but other
            % code in the Bebop object may use pPos.X(4:6).
            if angles.valid
                obj.RobotObj.pPos.X(4) = angles.roll;
                obj.RobotObj.pPos.X(5) = angles.pitch;
                obj.RobotObj.pPos.X(6) = angles.yaw;
            end

            % -------------------------------------------------------------
            % 3. Original 3D trajectory generation logic
            % -------------------------------------------------------------
            switch obj.trajectory_mode
                case 'takeoff_land_test'
                    Xd = [0; 0; obj.hover_Z];
                    dXd = [0; 0; 0];

                case 'circle'
                    Xd = [obj.rX*cos(obj.w*t_atual); obj.rY*sin(obj.w*t_atual); obj.hover_Z];
                    dXd = [-obj.rX*obj.w*sin(obj.w*t_atual); obj.rY*obj.w*cos(obj.w*t_atual); 0];

                case 'setpoints'
                    setPointsX = [1, -1, -1, 1];
                    setPointsY = [1, 1, -1, -1];
                    ptIndex = mod(floor(t_atual / 5), 4) + 1;
                    Xd = [setPointsX(ptIndex); setPointsY(ptIndex); obj.hover_Z];
                    dXd = [0; 0; 0];

                case 'infinity'
                    Xd = [obj.rX*sin(obj.w*t_atual); obj.rY*sin(2*obj.w*t_atual); obj.hover_Z];
                    dXd = [obj.rX*obj.w*cos(obj.w*t_atual); 2*obj.rY*obj.w*cos(2*obj.w*t_atual); 0];

                otherwise
                    fprintf('[BEBOP CONTROL] Unknown trajectory_mode: %s. Hovering.\n', obj.trajectory_mode);
                    obj.hover();
                    return;
            end

            % -------------------------------------------------------------
            % 4. Original 3D proportional control logic
            % -------------------------------------------------------------
            error_pos = Xd - obj.RobotObj.pPos.X(1:3);
            u_linear = dXd + obj.Kp * error_pos;

            obj.RobotObj.pSC.Ud = zeros(6,1);
            obj.RobotObj.pSC.Ud(1:3) = u_linear;

            % -------------------------------------------------------------
            % 5. Emergency and command output
            % -------------------------------------------------------------
            if obj.RobotObj.pFlag.EmergencyStop ~= 0
                obj.EmergencyTriggered = true;
                return;
            end

            obj.RobotObj.rCommand();
        end

        function hover(obj)
            if isempty(obj.RobotObj)
                return;
            end
            obj.RobotObj.pSC.Ud = zeros(6,1);
            obj.RobotObj.rCommand();
        end

        function landAndStop(obj)
            if isempty(obj.RobotObj)
                fprintf('[BEBOP CONTROL] RobotObj is empty. Cannot land.\n');
                return;
            end

            fprintf('[BEBOP CONTROL] Braking and stabilizing...\n');
            obj.hover();
            pause(2.0);

            fprintf('[BEBOP CONTROL] Initiating landing...\n');
            for i = 1:obj.nLandMsg
                obj.RobotObj.pSC.Ud = zeros(6,1);
                obj.RobotObj.rLand();
                pause(0.1);
            end
        end
    end

    methods (Access = private)
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
                if isfield(final_angles, 'roll');  angles.roll  = final_angles.roll;  end
                if isfield(final_angles, 'pitch'); angles.pitch = final_angles.pitch; end
                if isfield(final_angles, 'yaw');   angles.yaw   = final_angles.yaw;   end
                if isfield(final_angles, 'timestamp'); angles.timestamp = final_angles.timestamp; end
                if isfield(final_angles, 'source'); angles.source = string(final_angles.source); end
                if isfield(final_angles, 'valid')
                    angles.valid = logical(final_angles.valid);
                else
                    angles.valid = all(isfinite([angles.roll, angles.pitch, angles.yaw]));
                end
            end
        end

        function angles = makeInvalidAngles(~)
            angles = struct();
            angles.roll = 0;
            angles.pitch = 0;
            angles.yaw = 0;
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
    end
end
