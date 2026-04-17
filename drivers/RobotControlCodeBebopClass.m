% =========================================================================
% ROBOT CONTROL SCRIPT FOR BEBOP DRONES
%
% Description:
% Object-oriented autonomous control script for Bebop. Designed to be
% instantiated by the MatlabMasterUWBControl script.
% =========================================================================

classdef RobotControlCodeBebopClass < handle
    properties
        RobotObj     
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Coordinate Selection
        use_uwb_for_control = false; 
        
        % Offsets (meters) of optitrack from 0,0,0 of robot
        x_offset = -0.03;
        y_offset = 0.009;
        z_offset = -0.135;
        
        % Configuration Flags
        trajectory_mode = 'takeoff_land_test'; % takeoff_land_test, infinity, setpoints, circle
        
        % Timers and Trajectory Variables
        rX = 1; % X radius movement (meters)
        rY = 1; % Y radius movement (meters)
        hover_Z = 1; % Target flight altitude (meters)
        T = 15; % amount of time per movement (seconds)   
        
        % Controller Tuning
        Kp = 0.8; % Proportional gain (higher = more aggressive steering)

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        t_start
        w
        nLandMsg = 5;

        EmergencyTriggered = false;
    end
    
    methods
        function obj = RobotControlCodeBebopClass()
            disp('[BEBOP] Initializing Drone...');
            try
                obj.RobotObj = Bebop(1,'B1');
                
                % Pre-allocate the state array to prevent index errors
                obj.RobotObj.pPos.X = zeros(12,1);
                
                obj.w = 2*pi/obj.T;
            catch ME
                disp('[BEBOP] Error loading Bebop classes:');
                disp(ME.message);
            end
        end
        
        function perform_movement(obj)
            disp('[BEBOP] Taking off...');
            obj.RobotObj.rTakeOff();
            
            % Allow the drone time to reach initial altitude before tracking
            pause(4); 
            
            disp('[BEBOP] Starting movement trajectory...');
            obj.t_start = tic; 
        end
        
        function update(obj, uwb_pos, opti_pos)
            if isempty(obj.t_start)
                obj.t_start = tic;
            end
            t_atual = toc(obj.t_start);
            
            % Base Coordinate Selection
            if obj.use_uwb_for_control
                current_pos = uwb_pos;
            else
                current_pos = opti_pos;
            end
            
            % Apply offsets
            current_pos(1) = current_pos(1) + obj.x_offset;
            current_pos(2) = current_pos(2) + obj.y_offset;
            current_pos(3) = current_pos(3) + obj.z_offset; 
            
            % Update global position
            obj.RobotObj.pPos.X(1:3) = current_pos(:);
            
            % 3D Trajectory Generation
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
            end
            
            % 3D Proportional Control Algorithm
            % Calculate position error [X, Y, Z]
            error_pos = Xd - obj.RobotObj.pPos.X(1:3);
            
            % Calculate required velocity (Feedforward + Correction)
            u_linear = dXd + obj.Kp * error_pos; 
            
            % Assign linear velocities to the 6-DOF command vector
            obj.RobotObj.pSC.Ud = zeros(6,1);
            obj.RobotObj.pSC.Ud(1:3) = u_linear;
            
            % Check Emergency Flags
            if obj.RobotObj.pFlag.EmergencyStop ~= 0 
                obj.EmergencyTriggered = true;
                return;
            end
            
            % Send Control Signals to ROS
            obj.RobotObj.rCommand();
        end
        
        function landAndStop(obj)
            disp('[BEBOP] Braking and stabilizing...');
            
            % Force hover to kill momentum
            obj.RobotObj.pSC.Ud = zeros(6,1);
            obj.RobotObj.rCommand();
            pause(2.0);
            
            disp('[BEBOP] Initiating Landing...');
            for i = 1:obj.nLandMsg
                obj.RobotObj.pSC.Ud = zeros(6,1);
                obj.RobotObj.rLand();
                pause(0.1);
            end
        end
    end
end






% % =========================================================================
% % ROBOT CONTROL SCRIPT FOR BEBOP DRONES
% % Author: Renzo Eisma
% % Date: 09/04/2026
% % Description:
% % ...
% % Features:
% % * ...
% % =========================================================================
% 
% classdef RobotControlCodeBebopClass < handle
%     properties
%         RobotObj     
%         JoyObj       
%         Angulo = 5;
%         AngMax = 30;
%         AngMin = 5;
%         AddAngFlag = false;
%         SubAngFlag = false;
%         EmergencyTriggered = false;
% 
%         % (Estimated) Offset center of robot from opti centerpoint (in
%         % meters)
%         x_offset = -0.03;
%         y_offset = 0.009;
%         z_offset = -0.135;
% 
%     end
% 
%     methods
%         function obj = RobotControlCodeBebopClass()
%             disp('[BEBOP] Initializing Drone and Joystick...');
%             try
%                 % It is assumed rosinit has already been called in the Master script
%                 obj.RobotObj = Bebop(1,'B1');
%                 obj.JoyObj = JoyControl;
%             catch ME
%                 disp('[BEBOP] Error loading Bebop classes:');
%                 disp(ME.message);
%             end
%         end
% 
%         function perform_movement(obj)
%             disp('[BEBOP] Taking off...');
%             obj.RobotObj.rTakeOff();
%             pause(5);
%         end
% 
%         function update(obj, uwb_pos, opti_pos)
%             % 1. Reset Control Signals
%             obj.RobotObj.pSC.Ud = zeros(6,1);
% 
%             % 2. Read Joystick
%             obj.RobotObj = obj.JoyObj.mControl(obj.RobotObj);
% 
%             button_joystick = obj.JoyObj.pDigital(2);
%             aumenta_angulo = obj.JoyObj.pDigital(6);
%             diminui_angulo = obj.JoyObj.pDigital(5);
% 
%             % 3. Process Angle Adjustments
%             if aumenta_angulo
%                 if obj.AddAngFlag == false
%                     if obj.Angulo < obj.AngMax
%                         obj.Angulo = obj.Angulo + 5;
%                         obj.AddAngFlag = true;
%                     end
%                 end
%             else
%                 obj.AddAngFlag = false;
%             end
% 
%             if diminui_angulo
%                 if obj.SubAngFlag == false
%                     if obj.Angulo > obj.AngMin
%                         obj.Angulo = obj.Angulo - 5;
%                         obj.SubAngFlag = true;
%                     end
%                 end
%             else
%                 obj.SubAngFlag = false;
%             end
% 
%             % 4. Apply Scaling and Command
%             cmd_percent = obj.Angulo / obj.AngMax;
%             obj.RobotObj.pSC.Ud = obj.RobotObj.pSC.Ud .* cmd_percent;
% 
%             % 5. Check Emergency Flags
%             if obj.RobotObj.pFlag.EmergencyStop ~= 0 || button_joystick ~= 0
%                 obj.EmergencyTriggered = true;
%                 return;
%             end
% 
%             % 6. Send to ROS
%             obj.RobotObj.rCommand();
%         end
% 
%         function landAndStop(obj)
%             disp('[BEBOP] Braking and stabilizing...');
% 
%             % Send zero velocity to force the drone to brake and hover
%             obj.RobotObj.pSC.Ud = zeros(6,1);
%             obj.RobotObj.rCommand();
% 
%             % Give the drone 2 seconds to lose momentum and level out
%             pause(2.0);
% 
%             disp('[BEBOP] Initiating Landing...');
%             for i = 1:5
%                 obj.RobotObj.pSC.Ud = zeros(6,1);
%                 obj.RobotObj.rLand();
%                 pause(0.1);
%             end
%         end
%     end
% end