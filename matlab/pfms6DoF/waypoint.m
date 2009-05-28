% waypoint.m
%
% DESCRIPTION:
% This function provides waypoint navigation of the UAS by controlling the 
% current waypoint and evaluating when a waypoint is reached. When a 
% waypoint is reached it increments the aircraft to the next desired 
% waypoint. This function is called within simulink. Once all waypoints are 
% reached the function specifies the initail waypoint and the process is 
% repeated. The waypoint data is detailed within the function.
%
% PFMS Project, 2009
% Nicholas Rutherford

% NOTES:
% None.

function [ y ] = waypoint( lat, long, wp )

% Settings
capture_dist = 100;
init_loc = [-27.9432  153.1392, 1000];

% Waypoint
way_pts = [ -27.9319  153.1212, 1100;
            -27.9413  153.1168, 1200;
            -27.9370  153.1338, 900;
            -27.9472  153.1475, 1000];

len = size(way_pts);
        
% Convert to ENU from waypoint initial location
[Xr, Yr, Zr] = llh2xyz(init_loc(1), init_loc(2), 0);

% Convert UAS location to ENU
[X, Y, Z] = llh2xyz(lat, long, 0);
[east_ac,north_ac,u] = xyz2enu(Xr, Yr, Zr, X, Y, Z);

% Convert current waypoint to ENU
[X, Y, Z] = llh2xyz(way_pts(wp,1), way_pts(wp,2), 0);
[east_wp,north_wp,u] = xyz2enu(Xr, Yr, Zr, X, Y, Z);

% Capture policy
range = norm([east_ac, north_ac]-[east_wp,north_wp]);

if range < capture_dist
    wp=wp+1;
    if wp > len(1)
        wp=1;
    end
    % Convert new waypoint to ENU    
    [X, Y, Z] = llh2xyz(way_pts(wp,1), way_pts(wp,2), 0);
    [east_wp,north_wp, u] = xyz2enu(Xr, Yr, Zr, X, Y, Z);
end

alt=way_pts(wp,3);

y = [east_ac, north_ac, east_wp, north_wp, alt, wp];