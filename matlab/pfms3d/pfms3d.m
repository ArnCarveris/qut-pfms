% pfms3d.m
%
% DESCRIPTION:
% This script generates a simple 3D trajectory prediction model for a UAS
% in the xyz plane for the required time to reach the specified waypoint.
% The model considers the maximum turning rate and decent/climb rate of the
% UAS and utilises P control for guidance.
%
% PFMS Project, 2009
% Nicholas Rutherford

% NOTES:
% None.

% Perform Maintenance
clear all
close all
clc

%% Script Settings
% Control Settings
prediction_horizon = 10000; % Prediction Horizon
sampling_interval = 1;      % Sampling time
K = 4;                      % Proportional control gain for heading control
J = 0.05;                   % Proportional control gain for altitude control                   
prox = 25;                  % Assign simulation prox required to complete navigation
tolerance = 1E-5;           % Control Tolerance

% UAS Settings
init_loc_UAS = [0, 0 0];                                        % Initial UAS location
init_head_UAS = pi/2;                                           % Initial UAS heading (radians)
vel_UAS = 0.514444444*53;                                       % Velocity in xy plane (m/s) (Flamingo)
a_n_max = (0.514444444*53)^2/((0.514444444*53)/deg2rad(15));    % Maximum normal accelleration of the UAS (Flamingo)
v_z_max = 300*0.3048/60;                                        % Maximum climb rate (m/s) (Flamingo) (1.5240)

% Waypoint Data
waypoint = [ 2000, -3000, 200 ];  % Initial target location

% Write out intial states to console
init_cart_r = waypoint - init_loc_UAS;                                   % Initial cart range
[init_LOS, init_pol_range] = cart2pol(init_cart_r(1), init_cart_r(2));   % Initial LOS & polar range

% Calculate intial loop values required for prediction
Ts = 0:sampling_interval:prediction_horizon;    % Sampling Vector
loc_UAS = init_loc_UAS;                         % Set the loc of UAS
alpha = init_head_UAS;                          % Set heading angle
LOS_temp = init_LOS;                            % Set LOS value
temp_pol_range = init_pol_range;                % Set polar range

% DEBUG Settings
DEBUG = 0;                  % Assign debugging mode (1), Disable (0)
test_break_count = 10000;   % Assign loop break count withing DEBUG mode

% Generate variable storage vectors for plotting
loc_UAS_v = [];
a_n_v = [];
alpha_v = [];
alpha_dot_v = [];
LOS_v = [];
vel_c_v = [];
v_z_v = [];

% Simulation control variables
count = 0;                  % Assign loop counter
at_waypoint = 0;            % Assign control loop boolean value

%% Script
% Begin prediction loop
while (~at_waypoint)   
    %% CONTROL XY    
    % Calculate the range in cartesian coordinates
    cart_r = waypoint - loc_UAS;
                  
    % Calculate LOS and Range in polar coordinates
    [ LOS, pol_range ] = cart2pol(cart_r(1), cart_r(2));
    
    % Find closing velocity
    vel_c = (temp_pol_range-pol_range)/sampling_interval;
    
    % Correct loss for quadrant
    if LOS < -pi/2
        LOS = LOS + 2*pi;
    end
     
    % Calculate rate of change of LOS                      
    alpha_dot = LOS - alpha/sampling_interval;
    
    % Apply control gain    
    a_n = K*alpha_dot;     
    
    % Correct for a_n conditions
    if abs(a_n) < tolerance
        a_n = 0;        
    elseif a_n > a_n_max
        a_n = a_n_max; 
    elseif a_n < -a_n_max
        a_n = -a_n_max;
    end
    
    % If a_n is not 0, fly about a radius due to a_n
    if a_n ~= 0        
        % Find the radius the UAS is to fly about
        r = vel_UAS^2/a_n;

        % Find change in heading of UAS (angle at centre of circle)
        d_theta = (vel_UAS/sampling_interval)/r;

        % Use cosine rule to determine length of change of position
        d_length = sqrt(r^2 + r^2 - 2*r*r*cos(d_theta));

        % New position is d_theta/2 from old position/velocity
        d_position =  [(d_length*cos(alpha+(d_theta/2))) (d_length*sin(alpha+(d_theta/2))) 0];         

        % Assign new heading and UAS location
        alpha = alpha + d_theta;            % Update heading angle of UAS     
        loc_UAS = loc_UAS + d_position;     % Find next UAS location       
    else
        % Otherwise fly along heading         
        d_position = [vel_UAS*cos(alpha)*sampling_interval vel_UAS*sin(alpha)*sampling_interval 0];        
        loc_UAS = loc_UAS + d_position;
    end      
    
    %% CONTROL Z
    % Calculate the altitude difference
    delta_alt = cart_r(3);
    
    % Calculate the amount of velocity to dedicate to Z
    v_z = delta_alt*J;
    
    % Ensure the velocity component is not maxiumum ROC        
    if v_z > v_z_max
        v_z = v_z_max; 
    elseif v_z < -v_z_max
        v_z = -v_z_max;
    end
    
    % Calculate the altitude gain
    loc_UAS(3) = loc_UAS(3) + v_z*sampling_interval;
    
    %% MAINTENANCE
    
    % Aircraft Dynamic Vectors for plotting
    loc_UAS_v = [loc_UAS_v; loc_UAS];       % Create UAS Loction vector
    alpha_dot_v = [alpha_dot_v, alpha_dot]; % Create alpha_dot vector
    a_n_v = [a_n_v, a_n];                   % Create normal accelleration vector
    alpha_v = [alpha_v, alpha];             % Create alpha_dot vector
    LOS_v = [ LOS_v, LOS ];                 % Create LOS vector
    vel_c_v = [vel_c_v, vel_c];             % Create closing velocity vector
    v_z_v = [ v_z_v, v_z ];                 % Create climb velocity vector
    
    % Break from loop if at waypoint
    loc2_UAS = [ loc_UAS(1) loc_UAS(2)];
    waypoint2 = [ waypoint(1) waypoint(2)];
    %prox_UAS = norm(loc2_UAS - waypoint2);    
    prox_UAS = norm(loc_UAS - waypoint); 
    if prox_UAS < prox
        display('Aircraft Reached Waypoint');
        break;                
    end
    
    % Save the required updated variables
    LOS_temp = LOS;             % Store previous LOS of UAS 
    temp_pol_range = pol_range; % Store previous polar range
    count = count+1;            % Increment count    
    
    if DEBUG        
        % Display debug values       
        disp('DEBUG_vals = [count, cart_r, LOS, pol_range, vel_c, alpha_dot, a_n, alpha, loc_UAS]'),
        DEBUG_vals = [count, cart_r, LOS, pol_range, vel_c, alpha_dot, a_n, alpha, loc_UAS]        
        % Break at assigned count
        if count == test_break_count
            break;
        end  
    end    
end

% Write count to console
Iterations = count
Sampling_Time = sampling_interval

%/*************************************
% Plot the UAS and WAYPOINT locations
%/*************************************
figure(1)  
plot3(loc_UAS_v(:,1), loc_UAS_v(:,2), loc_UAS_v(:,3),'r')
hold on
plot3(waypoint(1), waypoint(2), waypoint(3), 'bx')
plot3(init_loc_UAS(1), init_loc_UAS(2), init_loc_UAS(3), 'kx')
xlabel('Position X'), ylabel('Position Y');
grid on;
axis('equal')
title('3D Trajectory');

%/*************************************
% Plot the UAS and DYNAMICS
%/*************************************
figure(2)
plot(a_n_v)
xlabel('Iteration'), ylabel('Accelleration (Rad/s)');
title('Normal Acceletation over Simulation Duration');

figure(3)
plot(alpha_v)
xlabel('Iteration'), ylabel('Polar Heading (Rad)');
title('Polar Heading over Simulation Duration');

figure(4)
plot(alpha_dot_v)
xlabel('Iteration'), ylabel('ROC of Polar Heading (Rad/s)');
title('Rate of change of Polar Heading over Simulation Duration');

figure(5)
plot(LOS_v)
xlabel('Iteration'), ylabel('LOS (Rad)');
title('LOS over Simulation Duration');

figure(6)
plot(vel_c_v)
xlabel('Iteration'), ylabel('Closing Velocity (m/s)');
title('Closing Velocity over Simulation Duration');

figure(7)
plot(v_z_v)
xlabel('Iteration'), ylabel('Climb Velocity (m/s)');
title('Climb Velocity over Simulation Duration');