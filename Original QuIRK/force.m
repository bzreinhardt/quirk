classdef force < hgsetget
    properties
        body1;
        body2;
        
        F;
        T;
        
        frame = 0;
    end %properties
    
    methods
        
        %% CONSTRUCTOR
        function fc = force(bod1, bod2, F, T, frame)
            % FORCE creates an inter-body force object for multibody
            % simulations. Syntax:
            %
            %   fc = force(body1, body2, forceFn, torqueFn, frame)
            %
            % Required inputs
            %
            %   body1:      First body to join; force object exerts F,T on
            %                   this body
            %   body2:      Second body to join; force object exerts -F,-T
            %                   on this body
            %   forceFn:    Function handle that returns force on body1
            %                   (body2 experiences reaction -force). Should
            %                   take arguments (body1, body2, time).
            %   torqueFn:   Function handle that returns torque on body1
            %                   (body2 experiences reaction -torque).
            %                   Should take arguments (body1, body2, time).
            %   frame:      choose one of the following:
            %                   0, 'N', 'inertial' (default)
            %                   1, 'B1', 'body1'
            %                   2, 'B2', 'body2'
            %
            % See also SPRING, BODY, JOINT, SENSOR, MBODY.
            
            %% --Set defaults
            fc.F = @(bod1, bod2, t) [0; 0; 0];
            fc.T = @(bod1, bod2, t) [0; 0; 0];
                
            %% --Parse inputs
            if nargin == 1 && isa(bod1, 'force')  % --If given just a force, return that force
                fc = bod1;
            elseif nargin < 2 && ~isa(bod1, 'force')
                error('FORCE:force:input', 'Force constructor requires two bodies.')
            elseif nargin >= 2   % --If given arguments, construct the specified force
                % Bodies
                if ~isa(bod1, 'body')
                    error('FORCE:force:input', 'First argument must be a body.')
                elseif ~isa(bod2, 'body')
                    error('FORCE:force:input', 'Second argument must be a body.')
                end
                fc.body1 = bod1;
                fc.body2 = bod2;

                % Function handles
                if nargin > 2
                    if ~isa(F, 'function_handle')
                        error('FORCE:force:input', 'Third argument must be a function handle.')
                    end
                    if nargin(F) ~= 3
                        error('FORCE:force:input', 'Force must be a function of (body1, body2, t).')
                      %TODO need to set up this check so that it doesn't
                      %require just bodies
%                     elseif all(size(F(body, body, 0)) ~= [3 1])
%                         error('FORCE:force:input', 'Force function must return a 3x1 matrix.')
                    end
                    fc.F = F;
                end
                if nargin > 3
                    if ~isa(T, 'function_handle')
                        error('FORCE:force:input', 'Fourth argument must be a function handle.')
                    end
                    if nargin(T) ~= 3
                        error('FORCE:force:input', 'Torque must be a function of (body1, body2, t).')
                        %TODO need to set up this check so that it doesn't
                      %require just bodies
%                     elseif all(size(T(body, body, 0)) ~= [3 1])
%                         error('FORCE:force:input', 'Torque function must return a 3x1 matrix.')
                    end
                    fc.T = T;
                end
                
                % Frame
                if nargin > 4
                    if ischar(frame)
                        frame = lower(frame);
                    end
                    switch frame
                        case {1 '1' 'b1' 'body1'}
                            fc.frame = 1;
                        case {2 '2' 'b2' 'body2'}
                            fc.frame = 2;
                        otherwise
                            fc.frame = 0;
                    end
                end
                
            else    % --Construct object with default properties

            end

        end
        
        %% DISPLAY
        function display(fc)
            % DISPLAY handles the printout of a force object when someone
            % types its name without ending the line in a semicolon.
            disp([inputname(1) ' = '])
            disp( '   |<===    QuIRK: inter-body force')
            disp(['      ===>|  Current value: ' num2str(fc.getForce')])
            disp(' ');
        end
        
        %% GETFORCE
        function Fval = getForce(fc, t)
            % GETFORCE returns the current force vector of the force
            % object, applied to body 1.
            %
            % See also GETTORQUE.
            if nargin < 2
                t = 0;
            end
            
            Q = eye(3);
            if fc.frame == 1
                Q = q2d(fc.body1.att);
            elseif fc.frame == 2
                Q = q2d(fc.body2.att);
            end
                
            Fval = Q * fc.F(fc.body1, fc.body2, t);
            
            
        end
        
        %% GETTORQUE
        function Tval = getTorque(fc, t)
            % GETTORQUE returns the current torque vector of the force
            % object, applied to body 1, in the inertial frame.
            %
            % See also GETFORCE.
            if nargin < 2
                t = 0;
            end
            
            Q = eye(3);
            if fc.frame == 1
                Q = q2d(fc.body1.att);
            elseif fc.frame == 2
                Q = q2d(fc.body2.att);
            end
            
            Tval = Q * fc.T(fc.body1, fc.body2, t);
        end
                
    end %methods
    
    events
        
    end %events
    
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
% Force class
%   Object describing an action-reaction interaction between a pair of body
%   objects.