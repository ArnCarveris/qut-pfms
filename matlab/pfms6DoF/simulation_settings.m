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
init_loc = [-26.458  151.3936, 1000];

% Waypoint
way_pts = [ -26.45809664139937  151.3963326734395, 1000;
            -26.47035910706417  151.3950924115017, 1000;
            -26.46726280552512  151.3902337706135, 1000;
            -26.45646302554365  151.3911983783632, 1000];     
        
save settings capture_dist init_loc way_pts