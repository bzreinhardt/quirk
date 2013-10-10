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
% DoublePendulum
%   This demo builds a complex system in the influence of gravity. Note the
%   use of solver options to tweak the behavior of MATLAB's ODE
%   functionality and the trace command to view the system trajectory.
%	The animate command will write an avi file to disk.

clear all
close all
clc

% Construct bodies
base = body([0 0 0.5], [0 0 0 1], 'shape', 'cube', ...
    'size', [2 2 1], 'color', 'm');
rod1 = body([0 2*sind(50) 2*cosd(50)], ...
    [sind(-50/2) 0 0 cosd(-50/2)], 'shape', 'cylinder', 'mass', 0.3, ...
    'size', [0.1 0.1 4], 'color', 'c');
rod2 = body([0 4*sind(50) 2+4*cosd(50)], ...
    [0 0 0 1], 'shape', 'cylinder', 'mass', 0.1, ...
    'size', [0.1 0.1 4], 'color', 'c');
bob = body([0 4*sind(50) 4+4*cosd(50)], ...
    [0 0 0 1], 'shape', 'cube', 'mass', 0.4, ...
    'size', [1 1 1], 'color', 'c', 'vel', [0.3 0 0]);

% Create joints
grnd = joint(base, 'ground');
fixbob = joint(rod2, bob, 'pt1', [0 0 2], 'pt2', [0 0 0], ...
    'type', 'fix');
hinge1 = joint(base, rod1, 'pt1', [0 0 -0.5], 'pt2', [0 0 -2], ...
    'type', 'sph', 'axis', [1 0 0]);
hinge2 = joint(rod1, rod2, 'pt1', [0 0 2], 'pt2', [0 0 -2], ...
    'type', 'sph', 'axis', [1 0 0]);

% Build multibody system
mb = mBody(base, rod1, rod2, bob, grnd, hinge1, hinge2, fixbob, ...
    'U', @(b)( 9.8*b.mass*b.pos(3) ), 'damping', 0.05 );

%% Solve system
figure(1)
solve(mb, [0:0.1:10], 'solver', 'ode45', 'tidy', true, 'odeopts', odeset('outputfcn', 'odeplot'));

%% Animate results
figure(2)
set(gcf, 'position', [200 200 480 640])
animate(mb, 'lit', true, 'bg', 'k', 'axis', [-5 5 -5 5 -5 9], 'filename', 'doublePendulum')

% Show system trajectory and energy
% figure(3)
% subplot(121)
% trace(mb, 'steps', 30);
% light
% set(gca, 'CameraPosition', [-100 0 0]);
% subplot(122)
% plotEnergy(mb);
