% analyse_pfms.m
%
% DESCRIPTION:
% This script generates plots of the PFMS simulation for all recorded data
% from the JSBSim flight model.
%
% In addition data can be saved to a .mat file.
%
% PFMS Project, 2009
% Nicholas Rutherford

% NOTES:
% None.

% Perform Maintenance
clc;
clear all;
close all;

% Initialise Conversion constant
FT_TO_M = 0.3048;

% Load data
data = dlmread('../output_file.csv',',',2,1);

%% Plot flight trajectory and associated waypoints
lat_col = 67;
long_col = 68;
h_agl_ft_col = 97;
lat_deg = data(:,lat_col);
long_deg = data(:,long_col);
h_agl_ft = data(:,h_agl_ft_col);
h_agl_m = h_agl_ft*FT_TO_M;

% Local system conversion
len = length(lat_deg);
east = zeros(1,len);
north = zeros(1,len);
up = zeros(1,len);

[Xr, Yr, Zr] = llh2xyz(lat_deg(1), long_deg(1), 0);
for ii=1:length(lat_deg)
    [X, Y, Z] = llh2xyz(lat_deg(ii), long_deg(ii), h_agl_m(ii));
    [e,n,u] = xyz2enu(Xr, Yr, Zr, X, Y, Z);
    east(ii) = e/1000;
    north(ii) = n/1000;
    up(ii) = u/1000;
end

figure(1)
plot3(east, north, up);
title('3D Plot of Flight Trajectory')
xlabel('East [km]'), ylabel('North [km]'), zlabel('Up agl [km]')
axis equal

figure(2)
plot(east, north)
title('2D Flight Trajectory')
xlabel('East [km]'), ylabel('North [km]')

figure(3)
subplot(3,1,1)
plot(east)
title('East Vs. Time')
xlabel('Time Steps'), ylabel('East [km]')
subplot(3,1,2)
plot(north)
title('North Vs. Time')
xlabel('Time Steps'), ylabel('North [km]')
subplot(3,1,3)
plot(up)
title('Up Vs. Time')
xlabel('Time Steps'), ylabel('Up [m]')

%% Plot aircraft dynamics TODO