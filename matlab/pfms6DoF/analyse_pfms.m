% analyse_pfms.m
%
% DESCRIPTION:
% This script generates plots of the PFMS simulation for all recorded data.
% In addition data can be saved to a .mat file.
%
% PFMS Project, 2009
% Nicholas Rutherford

% NOTES:
% None.


close all

% Generate a vector from the initial location to the first waypoint
initloctowp = [ flighttraj.signals.values(1,1),flighttraj.signals.values(2,2),flighttraj.signals.values(3,3);
                flighttraj.signals.values(1,4),flighttraj.signals.values(2,5),flighttraj.signals.values(3,6)];
len = size(flighttraj.signals.values);
            
% Plot flight trajectory
figure(1)
plot3(flighttraj.signals.values(:,1),flighttraj.signals.values(:,2),flighttraj.signals.values(:,3),'b')
hold on
plot3(flighttraj.signals.values(:,4),flighttraj.signals.values(:,5),flighttraj.signals.values(:,6),'g')
plot3(initloctowp(:,1),initloctowp(:,2),initloctowp(:,3),'g')
plot3(flighttraj.signals.values(1,1),flighttraj.signals.values(2,2),flighttraj.signals.values(3,3),'xr')
plot3(flighttraj.signals.values(len(1),1),flighttraj.signals.values(len(1),2),flighttraj.signals.values(len(1),3),'xk')
axis('equal')
title('Flight Trajectory')
hold off

% Plot plant input commands
figure(2)
subplot(3,1,1),plot(command.signals.values(:,1))
title('Bank Command - Heading'),xlabel('Iteration'),ylabel('Command')
subplot(3,1,2),plot(command.signals.values(:,2))
title('Throttle Command - Airspeed'),xlabel('Iteration'),ylabel('Command')
subplot(3,1,3),plot(command.signals.values(:,3))
title('Elevator Command - Altitude'),xlabel('Iteration'),ylabel('Command')

% Plot flight dynamics
figure(3)
subplot(3,1,1),plot(fdynamics.signals.values(:,1))
title('Airspeed'),xlabel('Iteration'),ylabel('Degrees')
subplot(3,1,2),plot(fdynamics.signals.values(:,2))
title('Sideslip'),xlabel('Iteration'),ylabel('Degrees')
subplot(3,1,3),plot(fdynamics.signals.values(:,3))
title('Angle of Attack'),xlabel('Iteration'),ylabel('Degrees')

% Plot Euler
figure(4)
subplot(3,1,1),plot(euler.signals.values(:,1))
title('Pitch Angle'),xlabel('Iteration'),ylabel('Degrees')
subplot(3,1,2),plot(euler.signals.values(:,3))
title('Bank Angle'),xlabel('Iteration'),ylabel('Degrees')
subplot(3,1,3),plot(euler.signals.values(:,3))
title('Heading'),xlabel('Iteration'),ylabel('Degrees')

% Save the flight data
%save flightdata flighttraj.signals.values command.signals.values fdynamics.signals.values euler.signals.values