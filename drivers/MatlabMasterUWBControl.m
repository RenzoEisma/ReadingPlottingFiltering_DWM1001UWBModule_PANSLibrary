% =========================================================================
% MATLAB MASTER UWB CONTROL
% Author: Renzo Eisma
% Date: 09/04/2026
%
% Description:
% Master control script, receives optitrack and uwb via udp from a python 
% script. Filters the UWB data. Sends UWB and Optitrack to robot control. 
% Initializes robot control script.
% =========================================================================

function MatlabMasterUWBControl()
    %% 0. SYSTEM CLEANUP & INIT
    clear classes; close all; clc;
    rosshutdown; pause(0.5); % Ensure old ROS connections are killed immediately to clear ghost nodes

    %% 1. SYSTEM CONFIGURATION
    SelectedRobot        = "Bebop2"; % "Bebop2", "Crazyflie", "Limu"
    
    ENABLE_ROS           = true; % Set to true to initialize ROS network
    ENABLE_ROBOT_CONTROL = true; % Set to true to send movement commands
    ENABLE_ACCEL_READ    = true; % Set to true to read IMU/Accelerometer data
    
    %% 2. HARDWARE SETUP
    disp('[MASTER] Opening UDP Ports...');
    try fclose(udpport("all")); catch; end % Ensure ports are free before opening
    u_uwb  = udpport("LocalHost", "127.0.0.1", "LocalPort", 5005); 
    u_opti = udpport("LocalHost", "127.0.0.1", "LocalPort", 5006); 
    
    uwb_filter = FilterUWB(0.1); 
    
    robot = []; 
    fid = -1; 
    ROS_INITIALIZED = false; 
    robot_is_moving = false;
    current_accel = [0,0,0];

    ros_ip = '192.168.0.100';

    disp('[MASTER] Waiting for UDP data to configure logging and ROS...');
    
    %% 3. MAIN LOOP VARIABLES
    T_exp = 2400; 
    T_run = 1/30; 
    movement_time = 10; 
    
    latest_uwb_raw = [0, 0, 0]; 
    latest_uwb_filt = [0, 0, 0]; 
    latest_opti = [0, 0, 0]; 
    
    t_exp = tic; 
    t_run = tic; 
    print_counter_uwb = 0;
    print_counter_opti = 0;
    print_counter_accel = 0;

    %% 4. HIGH SPEED EXECUTION LOOP
    try
        while toc(t_exp) < T_exp
            
            % A. UDP Reception and Setup
            if u_uwb.NumBytesAvailable > 0
                raw_data = read(u_uwb, u_uwb.NumBytesAvailable, "string");
                
                % Split the incoming data chunk by newlines
                packets = strsplit(raw_data, newline);
                
                % Find the most recent non-empty packet in the buffer
                latest_packet = "";
                for k = length(packets):-1:1
                    if strlength(strtrim(packets(k))) > 0
                        latest_packet = packets(k);
                        break;
                    end
                end
                
                if latest_packet ~= ""
                    data_parts = strsplit(latest_packet, ','); 
                    
                    if length(data_parts) >= 7
                        % Parse incoming Python config
                        array_time   = str2double(data_parts(1));
                        raw_x        = str2double(data_parts(2));
                        raw_y        = str2double(data_parts(3));
                        raw_z        = str2double(data_parts(4));
                        received_dir = strtrim(data_parts(5)); 
                        session_name = strtrim(data_parts(6)); 
                        
                        % Read the ROS flag directly from Python again
                        ENABLE_ROBOT_CONTROL = strcmpi(strtrim(data_parts(7)), '1');

                        latest_uwb_raw = [raw_x,raw_y,raw_z];
                        
                        % --- INITIALIZE ON FIRST VALID PACKET ---
                        if fid == -1
                            fprintf('[MASTER] First Packet Received: %s\n', latest_packet);
                            
                            % 1. File Logging Setup
                            csv_name = "[Log]_uwbFilteredMatlab_listener1_" + session_name + ".csv";
                            log_filename = fullfile(received_dir, csv_name);
                            fid = fopen(log_filename, 'a');
                            
                            if fid ~= -1
                                fprintf(fid, 'Time,Filt_X,Filt_Y,Filt_Z\n');
                                disp(['[MASTER] Logging Initialized: ', char(log_filename)]);
                                fileCleaner = onCleanup(@() fclose(fid));
                            else
                                disp('[MASTER] Warning: Could not open CSV at provided path. Using pwd.');
                                fid = fopen(csv_name, 'a');
                            end
                            
                            % 2. ROS Initialization
                            if ENABLE_ROS && ~ROS_INITIALIZED
                                disp('[MASTER] Initializing ROS Network...');
                                try
                                    try fclose(instrfindall); catch; end
                                    rosshutdown;
                                    pause(1);
                                    
                                    rosinit(ros_ip); 

                                    fprintf("[MASTER] Selected Robot: %s\n", SelectedRobot);
                                    
                                    if SelectedRobot == "Limu"
                                        robot = RobotControlCodeLimuClass();
                                    elseif SelectedRobot == "Bebop2"
                                        robot = RobotControlCodeBebopClass();
                                    elseif SelectedRobot == "Crazyflie"
                                        robot = RobotControlCodeLimuClass();
                                    end
                                    
                                    if ENABLE_ROBOT_CONTROL
                                        robot.perform_movement();
                                        robot_is_moving = true;
                                        disp('[MASTER] ROS Connection Successful. Robot Movement Started.');
                                    else
                                        disp('[MASTER] ROS Connection Successful. Robot Movement Disabled.');
                                    end
                                    
                                    ROS_INITIALIZED = true;
                                catch ME
                                    fprintf('[MASTER] ROS Error: %s\n', ME.message);
                                    ENABLE_ROS = false;
                                    ENABLE_ROBOT_CONTROL = false;
                                    ENABLE_ACCEL_READ = false;
                                end
                            end
                            
                            if fid == -1; fid = -2; end % Failsafe
                        end
                        
                        % Get latest Accelerometer data
                        if ENABLE_ACCEL_READ && ROS_INITIALIZED && ~isempty(robot)
                            if SelectedRobot == "Limu"
                                current_accel = robot.latest_accel;
                            elseif SelectedRobot == "Bebop2"
                                % Future Bebop2 accelerometer implementation
                            elseif SelectedRobot == "Crazyflie"
                                % Future Crazyflie accelerometer implementation
                            end
                            
                            print_counter_accel = print_counter_accel + 1;
                            if mod(print_counter_accel, 200) == 0
                                fprintf('[LIMU] Accel -> X: %.2f, Y: %.2f, Z: %.2f\n', current_accel(1), current_accel(2), current_accel(3));
                            end
                        end
    
                        % Filter and Update Data
                        [fx, fy, fz] = uwb_filter.process(raw_x, raw_y, raw_z);
                        latest_uwb_filt = [fx, fy, fz];
                        print_counter_uwb = print_counter_uwb + 1;
                        if mod(print_counter_uwb, 10) == 0
                            fprintf('[MATLAB] UWB Packet -> X: %.2f, Y: %.2f, Z: %.2f\n', fx, fy, fz);
                        end
                        
                        % Logging to CSV
                        if fid > 0
                            fprintf(fid, '%.3f,%.4f,%.4f,%.4f\n', array_time, fx, fy, fz);
                        end
                    end
                end
            end
            
            
            % B. OptiTrack Reception
            if u_opti.NumBytesAvailable > 0
                raw_str_opti = read(u_opti, u_opti.NumBytesAvailable, "string");
                parts_opti = strsplit(raw_str_opti(end), ','); 
                if length(parts_opti) >= 3
                    latest_opti = [str2double(parts_opti(1)), str2double(parts_opti(2)), str2double(parts_opti(3))];
                    
                    print_counter_opti = print_counter_opti + 1;
                    if mod(print_counter_opti, 200) == 0
                        fprintf('[MATLAB] Opti Packet -> X: %.2f, Y: %.2f, Z: %.2f\n', latest_opti(1), latest_opti(2), latest_opti(3));
                    end
                end
            end
            
            % C. Synchronous Robot Control (30Hz)
            if toc(t_run) > T_run
                t_run = tic;
                
                if ENABLE_ROBOT_CONTROL && ROS_INITIALIZED && ~isempty(robot)
                    % robot.update(latest_uwb_raw, latest_opti); % Use raw UWB data
                    robot.update(latest_uwb_filt, latest_opti); % Use filtered UWB data
                    
                    if toc(t_exp) > movement_time && robot_is_moving
                        disp('[MASTER] Movement Time Finished. Initiating Stop Sequence.');
                        robot.landAndStop();
                        robot_is_moving = false;
                        ENABLE_ROBOT_CONTROL = false; % Prevent further updates
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
        fprintf('[MASTER] CRASH: %s\n', ME.message);
    end
    
    %% 5. SAFE SHUTDOWN
    disp('[MASTER] Cleaning up resources...');
    if ~isempty(robot) && robot_is_moving
        robot.landAndStop();
    end
    
    try
        rosshutdown;
    catch
    end
    
    disp('[MASTER] Shutdown Complete.');
end


