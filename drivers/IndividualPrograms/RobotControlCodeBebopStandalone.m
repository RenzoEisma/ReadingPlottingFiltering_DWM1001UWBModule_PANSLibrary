%% INITIALIZATION
clear; close all; clc;

rosshutdown;

%% CLASS LOAD
try
    % Network
    rosinit('192.168.0.100');    
    B1 = Bebop(1,'B1');

    % Joystick
    J = JoyControl; % still need libraries for this

catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);

    rosshutdown;
    return;

end

%% VARIABLE INITIALIZATION
    % Time
    T_exp = 2400;       % Experience Duration
    T_run = 1/30;
 
%% TAKEOFF
TAKEOFF_ON = true;

if TAKEOFF_ON
    B1.rTakeOff;
    pause(5);
else
    disp('--- TakeOff Disabled');
end


%% TIME START
    t_exp = tic;
    t_run = tic;

    angulo = 5;
    ang_max = 30;
    ang_min = 5;

    add_ang_flag = false;
    sub_ang_flag = false;

    %% LOOP
try
    while toc(t_exp) < T_exp
        if toc(t_run) > T_run

        t_run = tic;
        t_now = toc(t_exp);

        %% JOYSTICK
        B1.pSC.Ud = zeros(6,1);
        
        B1 = J.mControl(B1);
        disp("B1.pSC.Ud:")
        disp(B1.pSC.Ud);
%         disp("Joystick Analog:")
%         J.pAnalog;
      
        button_joystick = J.pDigital(2);
        
        aumenta_angulo = J.pDigital(6);
        diminui_angulo = J.pDigital(5);

        if aumenta_angulo
            if add_ang_flag == false
                if angulo < ang_max
                    angulo = angulo + 5;
                    add_ang_flag = true;
                end
            end
        else
            add_ang_flag = false;
        end

        if diminui_angulo
            if sub_ang_flag == false
                if angulo > ang_min
                    angulo = angulo - 5;
                    sub_ang_flag = true;
                end
            end
        else
            sub_ang_flag = false;
        end
        
        disp("angulo: ")
        disp(angulo)

        cmd_percent = angulo/ang_max;

        B1.pSC.Ud = B1.pSC.Ud.*cmd_percent;


        
        %% EMERGENCY
        drawnow
        if B1.pFlag.EmergencyStop ~= 0 || button_joystick ~= 0
            disp('################ EMERGENCY ################');

            for i=1:5
                % Bebop Stop
                B1.pSC.Ud = zeros(6,1);
            
            end
            break;
        end

        %% SEND SIGNALS
            % Bebop
            disp("cmd_vels:")
            B1.pSC.Ud([1 2 3 6])
            B1.rCommand;
        end
    end
catch ME
disp('Bebop Landing through Try/Catch Loop Command');
B1.pSC.Ud = zeros(6,1);
disp('');
disp(ME);
disp('');
B1.rLand
end

%% FINAL STOP0
for i=1:5
    B1.pSC.Ud = zeros(6,1);
    B1.rLand;
end

%% ROS SHUTDOWN
rosshutdown;
disp("Ros Shutdown completed...");
    