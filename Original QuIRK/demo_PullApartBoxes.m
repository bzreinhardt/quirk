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
% PullApartBoxes
%   This demo constructs two boxes with a ball joint at a mutual corner and
%   puts them on a potential energy "hill" so that they fall apart. Then it
%   animates the result and displays a plot of the total system energy over
%   time.

clear all
close all
clc

% Construct bodies
b1 = body([0 -0.5 0], [0 0 0 1], 'shape', 'cube', 'color', 'b');
b2 = body([0 0.5 0], [0 0 0 1], 'shape', 'cube', 'color', 'r', 'torque', @(b)[0 0.1 0]');

% Create ball joint
ball = joint(b1, b2, 'pt1', [0.5 0.5 0.5], 'pt2', [0.5 -0.5 0.5], ...
    'type', 'rev', 'axis', [0 0 1]);

% Build multibody system
mb = mBody(b1, b2, ball, ... 
    'U', @(x,t)(-0.1*abs(x(2))), 'damping', 0.05 ); % 

% Solve system
solve(mb, [0 12]);

% Animate results
figure(1)
animate(mb, 'lit', true)

% Plot kinetic and potential energy as a function of time
figure(2)
[K U] = mb.plotEnergy;

% Plot system momentum as a function of time
figure(3)
[P H] = mb.plotMomentum;