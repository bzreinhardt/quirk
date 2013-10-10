function spr = spring(bod1, bod2, K, C, l0, frame)
% SPRING creates spring-damper force object for multibody
% simulations. Syntax:
%
%   spr = spring(body1, body2, K, C, length, frame)
%
%   body1: 	First body to join
%   body2: 	Second body to join
%   K:  	Linear stiffness value or matrix. Choose one:
%             	- Scalar radial stiffness value
%               Not yet implemented:
%             	- 2-elem vector of translation stiffness, torsion stiffness
%             	- 3x3 translation stiffness matrix
%             	- 6x6 stiffness matrix
%   C:   	Linear damping value or matrix. Choose one:
%            	- Scalar relative damping value
%               Not yet implemented:
%             	- 2-elem vector of translation damping, torsion damping
%             	- 3x3 translation damping matrix
%             	- 6x6 damping matrix
%   length: Equilibrium length of spring. Choose one:
%               - Scalar equilibrium distance
%               Not yet implemented:
%               - 3-elem translation equilibrium vector
%               - 6-elem transtation and rotation equilibrium vector
%   frame:	Relevant if stiffness or damping has been given as a 
%               matrix. Choose one of the following:
%               - 0, 'N', 'inertial' (default)
%               - 1, 'B1', 'body1'
%             	- 2, 'B2', 'body2'
%
% See also FORCE.

%---------------
% THINGS TO ADD
%
% (1) 3x3 stiffness, damping
% (2) 6x6 (7x7?) stiffness, damping
% (3) displaced attachment points from centers of mass

%% Set defaults
if nargin < 6
    frame = 0;
end
if nargin < 5
    l0 = 0;
end
if nargin < 4
    C = 0;
end
if nargin < 3
    K = 0;
end
if nargin < 2
    error('MULTIBODY:spring:inputs', 'Spring must have two bodies as inputs.')
end

%% Argument checking
if ~isa(bod1, 'body')
    error('MULTIBODY:spring:inputs', 'First argument to spring must be a body.')
end
if ~isa(bod2, 'body')
    error('MULTIBODY:spring:inputs', 'Second argument to spring must be a body.')
end
if ~isnumeric(l0)
    error('MULTIBODY:spring:inputs', 'Length must be numeric.')
end
if ~isnumeric(K)
    error('MULTIBODY:spring:inputs', 'Stiffness must be numeric.')
end
if ~isnumeric(C)
    error('MULTIBODY:spring:inputs', 'Damping must be numeric.')
end
if ~ischar(frame) && ~isnumeric(frame)
    error('MULTIBODY:spring:inputs', 'Frame must be numeric or a string.')
end
if ischar(frame)
    frame = lower(frame);
end
switch frame
    case {1 '1' 'b1' 'body1'}
        frame = 1;
    case {2 '2' 'b2' 'body2'}
        frame = 2;
    otherwise
        frame = 0;
end

if all(size(l0) == [1 3]) || all(size(l0) == [1 6])
    l0 = l0';
end

if numel(l0) ~= 1 && numel(l0) ~= 3
    error('Spring not yet implemented for length with other than 1 or 3 elements')
end
if numel(K) ~= 1 && numel(C) ~= 1
    error('Spring not yet implemented for K and C other than scalars')
end

%% Do stuff
if numel(l0) == 1
    if numel(K) == 1 && numel(C) == 1
        spr = force(bod1, bod2, ...
            @(b1,b2,t) -K*(b1.pos - b2.pos)*(1 - l0/norm(b1.pos - b2.pos)) - C*(b1.vel - b2.vel), ...
            @(b1,b2,t) [0; 0; 0], ...
            frame);
    end
end

if numel(l0) == 3
    if numel(K) == 1 && numel(C) == 1
        spr = force(bod1, bod2, ...
            @(b1,b2,t) -K*(b1.pos - b2.pos - l0) - C*(b1.vel - b2.vel), ...
            @(b1,b2,t) [0; 0; 0], ...
            frame);
    end
end

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
% Spring function
%   Shortcut function for creating a force object that represents a simple
%   spring-damper.