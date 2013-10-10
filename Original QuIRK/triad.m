function [out1 out2 out3] = triad(in1, in2)
% TRIAD constructs three mutually perpendicular unit vectors from either
% one or two input vectors. Syntax:
%
%   [out1 out2 out3] = triad(in1, in2)


if ~isnumeric(in1)
    error('triad:input:numeric', 'Triad requires numeric vector inputs.')
elseif all(size(in1) == [1 3])
    in1 = in1';
elseif numel(in1) ~= 3
    error('triad:input:size', 'Triad requires 3x1 vector inputs.')
elseif all(in1 == 0)
    error('triad:input:zero', 'Triad requires nonzero vector inputs.')
end
if nargin > 1 && ~isnumeric(in2)
    error('triad:input:numeric', 'Triad requires numeric vector inputs.')
elseif nargin > 1 && all(size(in2) == [1 3])
    in1 = in1';
elseif nargin > 1 && numel(in2) ~= 3
    error('triad:input:size', 'Triad requires 3x1 vector inputs.')
elseif nargin > 1 && all(in2 == 0)
    error('triad:input:zero', 'Triad requires nonzero vector inputs.')
end

if nargin > 1 && in1'*in2 ~= 0
    warning('triad:input:perp', 'Input vectors not perpendicular, using first input only.')
    clear in2
end

out1 = in1/norm(in1);

if nargin == 2
    out2 = in2/norm(in2);
    out3 = crs(in1)*in2;
    out3 = out3/norm(out3);
    return
else
    in2 = rand*in1 + [(2*rand-1) randn 1/rand]';
    
    out2 = crs(in1) * in2;
    out2 = out2/norm(out2);
    out3 = crs(in1) * out2;
    out3 = out3/norm(out3);
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
% Triad function
%   Helper function to construct three mutually perpendicular vectors from
%   one or two input vectors.