%   _____         %
%  /\\\\\\        % QuIRK :: Quaternion-state Interface
% /  \\\\\\       %                                 for Rigid-body Kinetics
% \  /####/       %
%  \/####/        %         A multibody dynamics package for Matlab 2009+
%        o_____   %
%        |\    \  % Author: Joseph Shoer
%        | \____\ %         Space Systems Design Studio
%        \ |    | %         Cornell University 
%         \|____| %         jps87 (at) cornell (dot) edu
%
% QuIRK is an interactive Matlab command line interface for constructing
% systems of rigid bodies and simple joint constraints, solving the
% equations of motion of those systems when subject to various force
% expressions, displaying and animating solved systems, and extracting
% information about the state history and energetics of those systems.
%
% QuIRK uses the Udwadia-Kalaba pseudoinverse method for constructing
% equations of motion for constrained systems, augmented for singular mass
% matrices. For details, see:
%
%   Udwadia, F. and Kalaba, R. Analytical Dynamics. Cambridge University
%       Press, 1996.
%
%   Udwadia, F. and Phohomsiri, P. "Explicit equations of motion for 
%       constrained mechanical systems with singular mass matrices and 
%       applications to multi-body dynamics." Proc. R. Soc. A, vol. 462, 
%       2006, p. 2097-2117.
%
% FourBarLinkage
%   This demo builds a four-bar linkage out of four rectangular bodies and
%   four hinges. It also includes a sensor object, which causes the
%   simulation to stop once the mechanism has opened, preventing it from
%   going to the point where the bodies would collide.

clear all
close all
clc

% Create individual bodies
angle = 1; % Initial opening angle of the mechanism
north = body([(1+tand(angle)/2)*cosd(angle) (1+1/(2*tand(angle)))*sind(angle) 0],...
    [0 0 sind(-angle/2) cosd(-angle/2)], 'shape', 'cube', 'color', 'b', ...
    'size', '2x1x1');
east = body([(1+tand(angle)/2)*cosd(angle) -(1+1/(2*tand(angle)))*sind(angle) 0],...
    [0 0 sind(angle/2) cosd(angle/2)], 'shape', 'cube', 'color', 'r', ...
    'size', '2x1x1');
south = body([-(1+tand(angle)/2)*cosd(angle) -(1+1/(2*tand(angle)))*sind(angle) 0],...
    [0 0 sind(-angle/2) cosd(-angle/2)], 'shape', 'cube', 'color', 'g', ...
    'size', '2x1x1');
west = body([-(1+tand(angle)/2)*cosd(angle) (1+1/(2*tand(angle)))*sind(angle) 0],...
    [0 0 sind(angle/2) cosd(angle/2)], 'shape', 'cube', 'color', 'm', ...
    'size', '2x1x1');

% Create hinges between bodies
NE = joint(north, east, 'pt1', [1 -0.5 0], 'pt2', [1 0.5 0], ...
    'type', 'rev', 'axis', [0 0 1]);
SE = joint(south, east, 'pt1', [1 0.5 0], 'pt2', [-1 0.5 0], ...
    'type', 'rev', 'axis', [0 0 1]);
SW = joint(south, west, 'pt1', [-1 0.5 0], 'pt2', [-1 -0.5 0], ...
    'type', 'rev', 'axis', [0 0 1]);
NW = joint(north, west, 'pt1', [-1 -0.5 0], 'pt2', [1 -0.5 0], ...
    'type', 'rev', 'axis', [0 0 1]);

% Create the sensor object
senSW = sensor(0.1, south, [-1.25 -0.25 0], west, [-1.25 0.25 0], 'm/s');

% Build multibody system
mb = mBody(north, south, east, west, NE, SE, NW, SW, senSW, ...
    'U', @(x,t)(-0.1*abs(x(2))));

% Solve system, stopping when the system trips the sensor
solve(mb, [0 6], 'sense', true);

% Animate results
figure(1)
animate(mb, 'lit', true)