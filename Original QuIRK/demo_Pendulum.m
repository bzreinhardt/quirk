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
% Pendulum
%   This demo builds a pendulum that swings in the influence of gravity. It
%   shows how to use 'ground' and 'fixed' joints to build complex bodies
%   about of smaller primitives. It also finds and spits out the state of
%   the system at equilibrium.

clear all
close all
clc

% Construct bodies
base = body([0 0 0.5], [0 0 0 1], 'shape', 'cube', ...
    'size', '2x2x1', 'color', 'm');
rod = body([0 4.9*sind(30) -4.9*cosd(30)], ...
    [sind(30/2) 0 0 cosd(30/2)], 'shape', 'box', 'mass', 0.1, ...
    'size', '0.1x0.1x9.8', 'color', 'b');
bob = body([0 9.8*sind(30) -9.8*cosd(30)], ...
    [sind(30/2) 0 0 cosd(30/2)], 'shape', 'box', ...
    'size', '1x1x1', 'color', 'b');

% Create joints
grnd = joint(base, 'ground');
fixbob = joint(rod, bob, 'pt1', [0 0 -4.9], 'pt2', [0 0 0], ...
    'type', 'fix');
hinge = joint(base, rod, 'pt1', [0 0 -0.5], 'pt2', [0 0 4.9], ...
    'type', 'hinge', 'axis', [1 0 0]);

% Build multibody system
mb = mBody(base, rod, bob, grnd, fixbob, hinge, ...
    'U', @(b)( 9.8*b.mass*b.pos(3) ), 'damping', 0.1 ); %@(x,t)( 9.8*x(3) )

% Solve system
solve(mb, [0 100], 'tidy', true, 'odeopts', odeset('outputfcn', @odeplot));

% Animate results
figure(1)
animate(mb, 'lit', true, 'axis', [-5 5 -5 5 -10 1])

% Find equilibrium position
figure(2)
x0 = mb.findeq('mode', 'f');
mb.draw('state', x0, 'lit', true, 'snap', true)
title('Pendulum Equilibrium')