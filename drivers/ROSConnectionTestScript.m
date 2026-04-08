% =========================================================================
% STANDALONE ROS TEST SCRIPT (OPEN LOOP)
%
% Description:
% This script bypasses all localization sensors (OptiTrack/UWB).
% It connects directly to the ROS Master and commands the robot to 
% spin in place for 10 seconds to verify the ROS-to-Robot connection.
% =========================================================================

clear; close all; clc;

%% 1. SYSTEM CLEANUP & ROS INIT
try
    objs = [udpportfind, serialportfind];
    if ~isempty(objs), delete(objs); end
catch
end

if license('test', 'Robot_System_Toolbox')
    rosshutdown;
    rosinit('192.168.0.100'); % ENSURE THIS IS YOUR CORRECT ROS MASTER IP
    pub = rospublisher('/B7/cmd_vel','geometry_msgs/Twist');
    msg = rosmessage(pub);
else
    error('ROS Toolbox is required to run this test script.');
end

%% 2. HARDWARE / GUI INIT
% Initialize Joystick control (wrapped in a try-catch in case it's unplugged)
try
    J = JoyControl;
    has_joystick = true;
    disp('Joystick connected.');
catch
    has_joystick = false;
    disp('No Joystick found. Relying on GUI Stop.');
end

% Creates a UI button to manually stop the robot
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'EMERGENCY STOP', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300], ...
                         'BackgroundColor', 'r', ...
                         'FontSize', 24);

%% 3. VARIABLES
T_run = 1/30;       % 30 Hz Control Loop
t_exp = tic;
t_run = tic;
nLandMsg = 10;      % Number of stop messages to send on exit

disp('Starting Open-Loop Test... Robot should rotate.');

%% 4. MAIN CONTROL LOOP
while 1
    if toc(t_run) > T_run 
        t_run = tic;
        t_atual = toc(t_exp);
        
        % --- A. SET OPEN-LOOP VELOCITIES ---
        % Command the robot to rotate in place
        linear_vel = 0.0;    % m/s
        angular_vel = 0.4;   % rad/s
        
        % Automatically stop the test after 10 seconds
        if t_atual > 10
            disp('10 seconds reached. Auto-stopping.');
            btnEmergencia = 1; 
        end

        % --- B. JOYSTICK OVERRIDE ---
        button_joystick = 0;
        if has_joystick
            mRead(J);
            % If joystick is moved, override the automatic rotation
            if abs(J.pAnalog(2)) > 0.1 || abs(J.pAnalog(5)) > 0.1 || abs(J.pAnalog(3)) > 0.1
                linear_vel = -J.pAnalog(5); 
                angular_vel = J.pAnalog(3);  
            end     
            button_joystick = J.pDigital(2);
        end
        
        % --- C. SEND COMMAND ---
        msg.Linear.X = linear_vel;
        msg.Angular.Z = angular_vel;
        send(pub,msg);
         
        % --- D. EMERGENCY HANDLING ---
        drawnow % Required to register GUI button clicks
        
        if btnEmergencia ~= 0 || button_joystick ~= 0
            disp('Stop command received. Exiting loop.');
            break;
        end   
    end
end

%% 5. SHUTDOWN SEQUENCE
disp('Sending stop signals...');
for i = 1:nLandMsg
    msg.Linear.X = 0;
    msg.Angular.Z = 0;
    send(pub,msg);  
    pause(0.05);
end     

rosshutdown;
disp('Test Complete. ROS Shut Down.');