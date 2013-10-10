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
% TwoStageLinkage
%   This demo builds a four-bar linkage as in the FourBarLinkage demo, with
%   a sensor that trips once the mechanism has (mostly) opened. Then it
%   changes the multibody system, removing a joint, so that the system can
%   be re-solved for further results. The demo saves the full state history
%   into the multibody system object and animates the result.

clear all
close all
clc

% Create individual bodies
angle = 10; % Initial opening angle of the mechanism
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
senSW = sensor(0.1, south, [-2 -0.25 0], west, [-2 0.25 0]);

% Build multibody system
mb = mBody(north, south, east, west, NE, SE, NW, SW, senSW, ...
    'U', @(x,t)(-0.1*abs(x(2))));

% Solve system, stopping when the system trips the sensor
solve(mb, [0 10], 'sense', true);

% Extract and save state history
t = mb.t;
state = mb.x;

% Cut the joint SW out of mb, clear the state history to prepare for a new
% solution, and solve again
mb.remove(SW);
mb.setnow;
solve(mb, [0 10], 'stop', 'coll');

% Overwrite system state with full time history
t = [t (t(end) + mb.t)];
state = [state mb.x];
mb.overwriteState(t, state);

% Animate results
animate(mb, 'lit', true)