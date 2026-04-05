function success = UWB_Matlab_Example(timestamp, raw_x, raw_y, raw_z)
    % persistent variables retain their values between Python calls
    persistent isInitialized pub_uwb msg_uwb state_x state_y state_z fid

    % =========================================================
    % 1. INITIALIZATION (Runs only once on the very first data point)
    % =========================================================
    if isempty(isInitialized)
        disp('[MATLAB] First point received. Initializing ROS and Files...');
        
%         % A. Start ROS
%         try
%             rosinit('192.168.0.100'); % Update with your ROS_MASTER IP
%         catch
%             disp('[MATLAB] ROS already running or failed to connect.');
%         end
%         pub_uwb = rospublisher('/uwb', 'geometry_msgs/Point');
%         msg_uwb = rosmessage(pub_uwb);

        % B. Get Configuration from Python Workspace
        try
            filter_type = evalin('base', 'uwb_filter_type');
            save_dir = evalin('base', 'save_dir'); 
        catch
            disp('[MATLAB] Warning: Could not read Python variables. Using defaults.');
            save_dir = pwd;
        end

        % C. Setup CSV File in the Measurement Folder
        filename = fullfile(save_dir, '[Log]_uwbFilteredMatlab_listener1.csv');
        fid = fopen(filename, 'a');
        fprintf(fid, 'Time,Filt_X,Filt_Y,Filt_Z\n');

        % D. Initialize Filter States
        state_x = raw_x; state_y = raw_y; state_z = raw_z;

        isInitialized = true;
    end

    % =========================================================
    % 2. FILTERING (Runs every time Python sends a point)
    % =========================================================
    
    % --- Replace this with your custom Kalman/Fusion Math ---
    % Example: Simple Low-Pass Filter (Alpha = 0.1)
    alpha = 0.1;
    filt_x = (1 - alpha) * state_x + alpha * raw_x;
    filt_y = (1 - alpha) * state_y + alpha * raw_y;
    filt_z = (1 - alpha) * state_z + alpha * raw_z;

    % Save the state so we have it for the next loop
    state_x = filt_x; 
    state_y = filt_y; 
    state_z = filt_z;

    % =========================================================
    % 3. OUTPUT & LOGGING
    % =========================================================
    
    % Publish to ROS
%     msg_uwb.X = filt_x;
%     msg_uwb.Y = filt_y;
%     msg_uwb.Z = filt_z;
%     send(pub_uwb, msg_uwb);

    % Log to CSV (using MATLAB's internal time for the timestamp)
    current_time = posixtime(datetime('now')); %unused
    fprintf(fid, '%.3f,%.4f,%.4f,%.4f\n', ...
            timestamp, filt_x, filt_y, filt_z);

    % Tell Python we finished successfully
    success = true; 
end