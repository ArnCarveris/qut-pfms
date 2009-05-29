% simulation_settings.m
%
% DESCRIPTION:
% This script generates a .mat file containing the settings  for the PFMS
% system including capture policy distance and waypoints.
%
% PFMS Project, 2009
% Nicholas Rutherford

% NOTES:
% None.

% Waypoint capture radius
capture_dist = 100;

% Initial location of the aircarft
init_loc = [-27.9432  153.1392, 1000];

% Waypoint
way_pts = [ -27.9319  153.1212, 1100;
            -27.9413  153.1168, 1200;
            -27.9370  153.1338, 900;
            -27.9472  153.1475, 1000];
        
save settings capture_dist init_loc way_pts