% heading.m
%
% DESCRIPTION:
% This function determines the required heading to reach a waypoint.
%
% PFMS Project, 2009
% Nicholas Rutherford

% NOTES:
% None.

function [ y ] = heading( east_ac, north_ac, east_wp, north_wp )

TH = atan2((east_wp-east_ac),(north_wp-north_ac));

req_head = rad2deg(TH);

if req_head < 0
    req_head = req_head +360;
end

y = req_head;