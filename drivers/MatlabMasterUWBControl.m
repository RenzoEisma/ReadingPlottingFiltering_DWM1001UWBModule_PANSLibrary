% =========================================================================
% MATLAB MASTER UWB CONTROL
% Author: Renzo Eisma
% Date: 09/04/2026
%
% Description:
% Master control script, receives optitrack and uwb via udp. Filters the 
% UWB data. Sends UWB or Optitrack to robot control. Initializes robot
% control script.
%
% Features:
% * ...
% =========================================================================

function MatlabMasterUWBControl()
    %% 1. SYSTEM CLEANUP & INIT
    clear classes; 
    close all; 
    clc;

    

    %% 2. HARDWARE & CLASS SETUP
    disp('[MASTER] Opening UDP Ports...');

    % Initialize UDP connections using matlab udp toolbox
    u_uwb  = udpport("LocalHost", "127.0.0.1", "LocalPort", 5005, "Timeout", 0.05);
    u_opti = udpport("LocalHost", "127.0.0.1", "LocalPort", 5006, "Timeout", 0.05);

    % Define robot control code
    % robot = RobotControlCodeBebopClass();
    % robot = RobotControlCodeBebopFigure8Class();
    robot = RobotControlCodeLimuClass();

    % Option for choosing if UWB or Optitrack is used as GT
    
    % Define Filter script
    uwb_filter = FilterUWB(0.1); 
    
    ros_ip = '192.168.0.100'; % Define ROS ip
    robot_movement = true; % Make the robot do the movement 
    
    % Initialize placeholders for dynamic configuration
    fid = -1; %...
    ROS_INITIALIZED = false; %...
    ENABLE_ROBOT_CONTROL; %...
    robot = []; %...
    
    disp('[MASTER] Waiting for UDP data to configure logging and ROS...');


    


    %% 3. MAIN LOOP VARIABLES
    T_exp = 2400; %...
    T_run = 1/30; %...
    movement_time = 10; % Amount of time robot performs movement
    
    latest_uwb_filt = [0, 0, 0]; % Variable for storing latest UWB data
    latest_opti = [0, 0, 0]; % Variable for storing latest Optitrack data
    
    t_exp = tic; %...
    t_run = tic; %...

    print_counter_uwb = 0; % Counter for printing every 10th UWB coordinate
    print_counter_opti = 0; % Counter for printing every 200th Optitrack coordinate

    robot_is_moving = false; %...
    


    %% 4. HIGH SPEED EXECUTION LOOP
    try
        while toc(t_exp) < T_exp
            
            % A. Asynchronous UDP Reading
            if u_uwb.NumBytesAvailable > 0
                raw_str = read(u_uwb, u_uwb.NumBytesAvailable, "string");
                data_parts = strsplit(raw_str(end), ','); 
                
                % Check for all 7 parameters: time, X, Y, Z, save_dir, session_name, ROS_flag
                if length(data_parts) >= 7
                    disp('[MASTER]' + raw_str);
                    array_time = str2double(data_parts(1));
                    raw_x = str2double(data_parts(2));
                    raw_y = str2double(data_parts(3));
                    raw_z = str2double(data_parts(4));
                    received_dir = strtrim(data_parts(5)); 
                    session_name = strtrim(data_parts(6)); 
                    %if strtrim(data_parts(7)) == "1"
                    if strcmpi(strtrim(data_parts(7)), '1')
                        ENABLE_ROBOT_CONTROL = true;
                    else
                        ENABLE_ROBOT_CONTROL = false;
                    end
                    
                    % --- INITIALIZE ON FIRST PACKET ---
                    if fid == -1
                        % Define file name
                        csv_name = "[Log]_uwbFilteredMatlab_listener1_" + session_name + ".csv";
                        log_filename = fullfile(received_dir, csv_name);
                        fid = fopen(log_filename, 'a');
                        
                        % Initialize CSV
                        if fid ~= -1
                            fprintf(fid, 'Time,Filt_X,Filt_Y,Filt_Z\n');
                            disp(['[MASTER] CSV Logging Initialized at: ', char(received_dir)]);
                            fileCleaner = onCleanup(@() fclose(fid));
                        else
                            disp('[MASTER] Warning: Could not open CSV at provided path. Using pwd.');
                            fid = fopen(csv_name, 'a');
                        end
                        
                        % Dynamically start ROS if Python requested it
                        if ENABLE_ROBOT_CONTROL && ~ROS_INITIALIZED
                            disp('[MASTER] Python enabled ROS. Initializing network...');
                            try
                                rosshutdown; 
                                pause(1);
                                rosinit(ros_ip); 
                                
                                if robot_movement
                                    robot.perform_movement();
                                    robot_is_moving = true;
                                end
                                ROS_INITIALIZED = true;
                            catch ME
                                disp('[MASTER] ROS Initialization Failed.');
                                ENABLE_ROBOT_CONTROL = false;
                            end
                        elseif ~ENABLE_ROBOT_CONTROL
                            disp('[MASTER] Robot control is DISABLED. Data Logging Mode only.');
                        end
                        
                        % Force fid to -2 if it somehow still failed, preventing infinite loops
                        if fid == -1
                            fid = -2;
                        end
                    end
                    
                    % Filter raw UWB coordinates in FilterUWB script
                    [fx, fy, fz] = uwb_filter.process(raw_x, raw_y, raw_z);
                    latest_uwb_filt = [fx, fy, fz];

                    % Print filtered UWB data every x times
                    print_counter_uwb = print_counter_uwb + 1;
                    if mod(print_counter_uwb, 10) == 0
                        fprintf('[MATLAB] UWB Packet -> X: %.2f, Y: %.2f, Z: %.2f\n', fx, fy, fz);
                    end
                    
                    % Write to CSV
                    if fid > 0
                        fprintf(fid, '%.3f,%.4f,%.4f,%.4f\n', array_time, fx, fy, fz);
                    end
                end
            end
            
            % Read Optitrack data
            if u_opti.NumBytesAvailable > 0
                raw_str = read(u_opti, u_opti.NumBytesAvailable, "string");
                data_parts = strsplit(raw_str(end), ','); 
                
                if length(data_parts) >= 3
                    latest_opti = [str2double(data_parts(1)), str2double(data_parts(2)), str2double(data_parts(3))];
                    
                    % Print Optitrack data every x times
                    print_counter_opti = print_counter_opti + 1;
                    if mod(print_counter_opti, 200) == 0
                        fprintf('[MATLAB] Opti Packet -> X: %.2f, Y: %.2f, Z: %.2f\n', latest_opti(1), latest_opti(2), latest_opti(3));
                    end
                end
            end
            
            % B. Synchronous Robot Control
            if toc(t_run) > T_run
                t_run = tic;
                
                if ENABLE_ROBOT_CONTROL && ROS_INITIALIZED
                    robot.update(latest_uwb_filt, latest_opti); 
                    % Exit the loop after 10 seconds of movement
                    if toc(t_exp) > movement_time
                        disp('[MASTER] Time limit seconds reached. Ending robot movement.');
                        robot.landAndStop();
                        robot_is_moving = false;
                        break;
                    end
                    if robot.EmergencyTriggered
                        disp('################ EMERGENCY ################');
                        break;
                    end
                end
                
                drawnow; 
            end
            
            pause(0.001); 
        end
        
    catch ME
        disp('[MASTER] Loop crashed. Initiating emergency failsafe.');
        disp(ME.message);
    end

    
    
    %% 5. SAFE SHUTDOWN
    try
        if fid ~= -1
            fclose(fid);
        end
    catch
    end
    
    if ENABLE_ROBOT_CONTROL && ROS_INITIALIZED
        if robot_is_moving
            robot.landAndStop();
        end
        rosshutdown;
    end
    
    disp('[MASTER] Shutdown completed.');
end



