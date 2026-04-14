% =========================================================================
% ROBOT CONTROL SCRIPT FOR WHEELED ROBOTS
%
% Description:
% This MATLAB script controls a wheeled robot (like Limu or Pioneer) using
% an OptiTrack system for localization and ROS for velocity commands.
%
% Trajectory Modes:
% The script includes three operational modes for trajectory generation:
% 1. Circular trajectory
% 2. Infinity pattern (Lissajous figure-eight)
% 3. Continuous set points
%
% To select a mode, navigate to the "TRAJECTORY GENERATION" section, 
% comment out the unused modes, and uncomment your desired mode.
%
% Features:
% * OptiTrack data acquisition
% * ROS integration for movement control
% * Joystick override support
% * GUI Emergency Stop button
% =========================================================================

% Supported robots: Limu, Pioneer

clear; close all; clc;
try
    fclose(instrfindall);
catch
    % Suppress error if no instruments are found
end
rosshutdown;


%% SYSTEM INITIALIZATION

% Load Classes
% Create OptiTrack object and initialize connection
OPT = OptiTrack;
OPT.Initialize;

% Create UDP objects for both sources
u_uwb = udpport("LocalHost", "127.0.0.1", "LocalPort", 5005);
u_opti = udpport("LocalHost", "127.0.0.1", "LocalPort", 5006);
% Define which source to use for control (Toggle this for your tests)
use_uwb_for_control = false;

% Initialize Joystick control
J = JoyControl;

% Network Initialization
rosinit('192.168.0.100');
pub = rospublisher('/L1/cmd_vel','geometry_msgs/Twist');
% pub = rospublisher('/RosAria/cmd_vel','geometry_msgs/Twist');
msg = rosmessage(pub);

% Initiate Pioneer/Robot classes
P1 = RPioneer(1,'RosAria',1);
idP = 1; % Pioneer/Bebop ID


%% EMERGENCY STOP GUI
% Creates a UI button to manually stop the robot
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'STOP', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);


%% VARIABLE INITIALIZATION

data = [];

% Timers
T_exp = 60;         % Total experiment time (seconds)
T_run = 1/30;       % Experiment sampling time (seconds)
t_run = tic;
t_exp = tic;
t  = tic;

% Trajectory variables
rX = 1;
rY = 1;
T = 10;             % Period of the trajectory

w = 2*pi/T;         % Angular frequency

pgains = [1.5 1 1.5 1]; 


%% MAIN CONTROL LOOP

while 1
    if toc(t_run) > T_run 
        t_run = tic;
        t_atual = toc(t_exp);
        

        %% TRAJECTORY GENERATION
        % Comment out the modes you do not wish to use.
        
        % Mode 1: Circle Trajectory
        % P1.pPos.Xd(1:2) = [rX*cos(w*t_atual); rY*sin(w*t_atual)];
        % P1.pPos.Xd(7:8) = [-rX*w*sin(w*t_atual); rY*w*cos(w*t_atual)]; 
        
        % Mode 2: Infinity Pattern (Figure-Eight / Lissajous)
        P1.pPos.Xd(1:2) = [rX*sin(w*t_atual); rY*sin(2*w*t_atual)];
        P1.pPos.Xd(7:8) = [rX*w*cos(w*t_atual); 2*rY*w*cos(2*w*t_atual)];

        % Mode 3: Continuous Set Points
        % Switches between 4 corner points every 5 seconds
        % setPointsX = [1, -1, -1, 1];
        % setPointsY = [1, 1, -1, -1];
        % ptIndex = mod(floor(t_atual / 5), 4) + 1;
        % P1.pPos.Xd(1:2) = [setPointsX(ptIndex); setPointsY(ptIndex)];
        % P1.pPos.Xd(7:8) = [0; 0]; 


        %% ODOMETRY & SENSORS
        
        % OptiTrack data acquisition
%        rb = OPT.RigidBody;
%        P1 = getOptData(rb(idP),P1);
        % P1.pPos.X([1:3 6])

        % UWB (Insert when done)

        % Angle calculations with UWB
        % Insert here when done

        %% ODOMETRY & SENSORS V2

        % 1. Read OptiTrack (Ground Truth)
        if u_opti.NumBytesAvailable > 0
            raw_str = read(u_opti, u_opti.NumBytesAvailable, "string");
            data = strsplit(raw_str(end), ','); % Get latest packet
            opti_pos = [str2double(data(1)), str2double(data(2)), str2double(data(3))];
        end
        
        % 2. Read UWB
        if u_uwb.NumBytesAvailable > 0
            raw_str = read(u_uwb, u_uwb.NumBytesAvailable, "string");
            data = strsplit(raw_str(end), ','); % Get latest packet
            uwb_pos = [str2double(data(1)), str2double(data(2)), str2double(data(3))];
            
            % 3. Apply your filter here (the code from UWB_Matlab_Example.m)
            alpha = 0.1;
            filt_uwb = (1 - alpha) * prev_filt + alpha * uwb_pos;
        end
        
        % 4. Select coordinate for the Control Algorithm
        if use_uwb_for_control
            P1.pPos.X = filt_uwb'; 
        else
            P1.pPos.X = opti_pos';
        end


        %% ROBOT CONTROL ALGORITHM
        
        % Feedback linearization for unicycle kinematics
        a = .15; % Look-ahead distance to avoid singularity
        psi = P1.pPos.X(6); % Current yaw angle
        
        % Rotation matrix / Decoupling matrix
        K = [cos(psi) -a*sin(psi);
             sin(psi)  a*cos(psi)];
         
        % Desired velocities and positions
        dXd = P1.pPos.Xd(7:8);
        Xd = P1.pPos.Xd(1:2);
        
        % Current positions
        X = P1.pPos.X(1:2);

        Kp = .5; % Proportional gain

        % Control law
        u = inv(K)*(dXd + Kp*(Xd - X));


        %% JOYSTICK CONTROL OVERRIDE
        
        mRead(J);
        % Check if joystick axes have significant input to override autonomous control
        if abs(J.pAnalog(2)) > 0.1 || abs(J.pAnalog(5)) > 0.1 || abs(J.pAnalog(3)) > 0.1
            u(1) = -J.pAnalog(5); % Linear velocity override
            u(2) = J.pAnalog(3);  % Angular velocity override
        end     

        button_joystick = J.pDigital(2);


        %% SENDING CONTROL SIGNALS

        msg.Linear.X = u(1);
        msg.Angular.Z = u(2);
        send(pub,msg);
         

        %% EMERGENCY HANDLING
        
        drawnow   
        if btnEmergencia ~= 0 || button_joystick ~= 0
            disp('Pioneer stopping by Emergency');

            % Send multiple zero-velocity commands to ensure the robot stops safely
            for i=1:nLandMsg
                msg.Linear.X = 0;
                msg.Angular.Z = 0;
                send(pub,msg);   
            end
            break;
        end   
        
        
        %% DATA LOGGING
       
        % Logging desired position, actual position, desired control, and time
        % data = [data; P1.pPos.Xd' P1.pPos.X' P1.pSC.Ud' P1.pSC.u_feito'    t_atual];
        data = [data; P1.pPos.Xd' P1.pPos.X' P1.pSC.Ud'    t_atual];

        % Columns:  1-12        13-24     25 26          27 28            29
            
    end
end


%% EMERGENCY OUTSIDE THE LOOP

% Failsafe to guarantee robot stops after exiting the loop
for i=1:nLandMsg
    msg.Linear.X = 0;
    msg.Angular.Z = 0;
    send(pub,msg);  
end     


%% SHUTDOWN CONTROL SIGNALS

for ii = 1:10
    msg.Linear.X = 0;
    msg.Angular.Z = 0;
    send(pub,msg);  
end


%% PLOT RESULTS

Xtil = data(:,1:12) - data(:,13:24);

figure();
hold on;
grid on;
plot(data(:,end),Xtil(:,1));
plot(data(:,end),Xtil(:,2));
title('Erro de Posição');
legend('Pos X','Pos Y');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
subplot(211)
hold on;
grid on;
plot(data(:,end),data(:,1));
plot(data(:,end),data(:,13));
xlabel('time $(s)$','interpreter','latex');
ylabel('$x (m)$','interpreter','latex');
legend('$x_{des}$','$x$','interpreter','latex');

subplot(212)
hold on;
grid on;
plot(data(:,end),data(:,2));
plot(data(:,end),data(:,14));
xlabel('time (s)','interpreter','latex');
ylabel('y (m)','interpreter','latex');
legend('$y_{des}$','$y$','interpreter','latex');