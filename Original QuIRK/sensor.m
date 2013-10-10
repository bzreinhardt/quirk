classdef sensor < hgsetget
    properties
        master = struct('body', {}, 'position', [0 0 0]');
        slave = struct('body', {}, 'position', [0 0 0]');
        mode = 'any'
        radius = 0.1;
        triggerfn = @(sen)disp(['Sensor triggered: ' num2str(sen.tripped)]);
        h = [];
    end
    
    properties (SetAccess = 'private')
        tripped = -1;
    end %properties

    methods

        %% CONSTRUCTOR
        function sen = sensor(varargin)
            % SENSOR creates a family of sensors for use in multibody
            % simulations. When two sensors in the same family get within a
            % specified radius of one another, they can trigger the
            % simulation to halt. See the "sense" option of MBODY/SOLVE.
            % Syntax:
            %
            %   sen = sensor(radius, mBody, mPosition, ...
            %                   sBody1, sPosition1, ..., flag)
            %   sen = sensor(radius, mBody, mPosition, ...
            %                   {sBody1, sPosition1, ...}, flag)
            %
            %   sen = sensor(radius, {mBody1, mPosition1, ...}, ...
            %                   sBody1, sPosition1, ..., flag)
            %   sen = sensor(radius, {mBody1, mPosition1, ...}, ...
            %                   {sBody1, sPosition1, ...}, flag)
            %
            % Required arguments
            %
            %   body_i:     Bodies on which to place sensors
            %   position_i: Position of sensor from body_i CM position, in
            %                   body_i's body coordinates
            %
            %   If you give the sensor constructor a list of bodies and
            %   positions, or a body/position pair and then a cell array of
            %   body/position pairs, then the FIRST body becomes the master 
            %   and all others become slaves. (First syntax shown above.)
            %
            %   If you provide the sensor constructor with a cell array of
            %   body/position pairs BEFORE either a list of body/position
            %   pairs or a list in a call array, then all the bodies in the
            %   first cell array become masters and all other become
            %   slaves. (Second syntax shown above.)
            %
            %   You must list at least one body with at least one
            %   corresponding position. (If you include only one
            %   body/position pair, the sensor will never trigger.)
            %
            % Optional arguments
            %
            %   radius  Separation between sensors in this family
            %               before they trigger (default = 0.1)
            %   'any'   If this flag is present at the end of the argument
            %               list, then the sensor group will operate in
            %               "any" mode. It will trigger when ANY sensor
            %               gets near any SLAVE sensor. (Default mode.)
            %   'm/s'   If this flag is present at the end of the argument
            %               list, then the sensor group will operate in
            %               "master/slave" mode. It will trigger when any
            %               master sensor gets near any slave sensor.
            %               (This mode evaluates faster.)
            %
            % Typically, the sensor has to be tripped for two ODE timesteps
            % in order to halt the simulation. Tweak the radius to achieve
            % this.
            %
            % See also TRIGGER, MBODY, MBODY/SOLVE, BODY, JOINT, FORCE.

            if nargin == 0
                sen.master = struct('body', body, 'position', [0 0 0]');
                sen.slave = struct('body', body, 'position', [0 0 0]');
            elseif nargin == 1
                sen = varargin{1};
            else
                % Process optional radius flag
                if isnumeric(varargin{1})
                    if ~isnumeric(varargin{1})
                        error('SENSOR:sensor:input', ...
                            'Sensor radius must be a number.')
                    end
                    sen.radius = varargin{1}(1);
                    varargin = varargin(2:end);
                end
                
                % Process optional mode flag
                if ischar(varargin{end})
                    switch lower(varargin{end})
                        case 'any'
                            sen.mode = 'any';
                        case 'm/s'
                            sen.mode = 'm/s';
                    end
                    varargin = varargin(1:(end-1));
                end
                
                if length(varargin) < 2
                    error('SENSOR:sensor:input', ...
                            'Not enough inputs to sensor constructor.')
                end
                
                if iscell(varargin{1})
                    masterIn = varargin{1};
                    varargin = varargin(2:end);
                elseif isa(varargin{1}, 'body') && isnumeric(varargin{2})
                    masterIn = varargin(1:2);
                    varargin = varargin(3:end);
                end
                
                if iscell(varargin{1})
                    slaveIn = varargin{1};
                elseif isa(varargin{1}, 'body') && isnumeric(varargin{2})
                    slaveIn = varargin;
                end
                
                % Process masters
                if numel(masterIn)/2 ~= floor(numel(masterIn)/2)
                    error('SENSOR:sensor:input', ...
                        'All bodies must be paired with positions.')
                end
                for i = 1:(numel(masterIn)/2)
                    if ~isa(masterIn{2*i-1}, 'body')
                        error('SENSOR:sensor:input', ...
                            'Sensor constructor requires body/position pairs.')
                    elseif ~isnumeric(masterIn{2*i})
                        error('SENSOR:sensor:input', ...
                            'Sensor positions must be numeric.')
                    elseif all(size(masterIn{2*i}) == [1 3])
                        masterIn{2*i} = masterIn{2*i}';
                    elseif numel(masterIn{2*i}) ~= 3
                        error('SENSOR:sensor:input', ...
                            'Sensor positions must be 3-element matrices.')
                    end
                    sen.master(i).body = masterIn{2*i-1};
                    sen.master(i).position = masterIn{2*i};
                end

                % Process slaves
                if numel(slaveIn)/2 ~= floor(numel(slaveIn)/2)
                    error('SENSOR:sensor:input', ...
                        'All bodies must be paired with positions.')
                end
                for i = 1:(numel(slaveIn)/2)
                    if ~isa(slaveIn{2*i-1}, 'body')
                        error('SENSOR:sensor:input', ...
                            'Sensor constructor requires body/position pairs.')
                    elseif ~isnumeric(slaveIn{2*i})
                        error('SENSOR:sensor:input', ...
                            'Sensor positions must be numeric.')
                    elseif all(size(slaveIn{2*i}) == [1 3])
                        slaveIn{2*i} = slaveIn{2*i}';
                    elseif numel(slaveIn{2*i}) ~= 3
                        error('SENSOR:sensor:input', ...
                            'Sensor positions must be 3-element matrices.')
                    end
                    sen.slave(i).body = slaveIn{2*i-1};
                    sen.slave(i).position = slaveIn{2*i};
                end
                
            end
            
            if isempty(sen.master(1).body) || isempty(sen.slave(1).body)
                warning('SENSOR:sensor:never', 'This sensor will never trigger.')
            end
        end
        
        %% DELETE
        function delete(sen)
            % DELETE performs clean-up actions before clearing a sensor 
            % from memory.
            erase(sen);
        end

        %% DISPLAY  
        function display(sen)
            % DISPLAY handles the printout of a sensor object when someone
            % types its name without ending the line in a semicolon.
            disp([inputname(1) ' = '])
            if strcmp( sen.mode, 'any' )
                disp('       /|   QuIRK: any-trigger sensor group')
            else
                disp('       /|   QuIRK: master/slave sensor group')
            end
            disp(['     [( |=@  With ' num2str(numel(sen.master)) ...
                ' master sensors'])
            disp(['    // \|     and ' num2str(numel(sen.slave)) ' slave sensors'])
            if sen.tripped > 0
                disp(['    ||        ' num2str(sen.tripped) ' sites triggered.'])
            elseif sen.tripped < 0
                disp('    ||       Never triggered.')
            else
                disp('    ||       Currently untriggered.')
            end
            disp(' ');
        end
        
        %% DRAW
        function h = draw(sen, varargin)
            % DRAW adds a graphic of a sensor to the current axes. Sensors
            % appear as colored triangles; empty when untriggered and
            % filled when triggered. The master of the sensor family points
            % upwards. Syntax:
            %
            %   h = draw(sensor)
            %   h = sensor.draw
            %
            % Optional properties
            %
            %   color   dbl/str Color of sensor
            %   snap    T/F     Graphic will not update as body changes if
            %                       this flag is true
            %
            % See also ERASE, MBODY\DRAW.
            
            %% --Parse inputs
            color = 'b';
            snap = false;
            
            plotargs = true(size(varargin));
            if numel(varargin)/2 ~= round(numel(varargin)/2)
                error('SENSOR:draw:input', 'All properties must be paired with values.')
            end
            for i = 1:2:(length(varargin))
                if ~ischar(varargin{i})
                    error('SENSOR:draw:input', 'Property designations must be strings.')
                end
                switch lower(varargin{i})
                    case {'color' 'col' 'c'}
                        color = varargin{i+1};
                        plotargs(i:(i+1)) = [false false];
                    case 'snap'
                        snap = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                end
            end
            
            %% --Erase old sensor graphics
            erase(sen);
            
            wasHeld = ishold;
            if ~wasHeld
                hold on
            end
            
            %% --Update sensors
            edgeCol = 'b';
            if sen.trigger(true)
                edgeCol = 'w';
            end
            
            %% --Draw master in filled/unfilled state
            for i = 1:numel(sen.master)
                pts = sen.master(i).body.pos + ...
                    ( q2d(sen.master(i).body.att) * sen.master(i).position );
                if sen.tripped > 0 %sen.trigger(true)
                    h = plot3(pts(1), pts(2), pts(3), ...
                        '^', 'color', edgeCol, 'MarkerFaceColor', color);
                else
                    h = plot3(pts(1), pts(2), pts(3), ...
                        '^', 'color', edgeCol, 'MarkerFaceColor', 'none');
                end
                if ~snap
                    sen.h = [sen.h h];
                end
                h = line([sen.master(i).body.pos(1); pts(1)], ...
                    [sen.master(i).body.pos(2); pts(2)], ...
                    [sen.master(i).body.pos(3); pts(3)], 'Color', color);
                if ~snap
                    sen.h = [sen.h h];
                end
            end
            
            %% --Draw slaves in filled/unfilled state
            for i = 1:numel(sen.slave)
                pts = sen.slave(i).body.pos + ...
                    ( q2d(sen.slave(i).body.att) * sen.slave(i).position );
                if sen.tripped > 0 %sen.trigger(true) > 0
                    h = plot3(pts(1), pts(2), pts(3), ...
                        'v', 'color', edgeCol, 'MarkerFaceColor', color);
                else
                    h = plot3(pts(1), pts(2), pts(3), ...
                        'v', 'color', edgeCol, 'MarkerFaceColor', 'none');
                end
                if ~snap
                    sen.h = [sen.h h];
                end
                h = line([sen.slave(i).body.pos(1); pts(1)], ...
                    [sen.slave(i).body.pos(2); pts(2)], ...
                    [sen.slave(i).body.pos(3); pts(3)], 'Color', color);
                if ~snap
                    sen.h = [sen.h h];
                end
            end
            
            if ~wasHeld
                hold off
            end
            
        end
        
        %% ERASE
        function erase(sen)
            % ERASE clears the graphical representation of a sensor from
            % all axes, if one exists. Syntax:
            %
            %   erase(sen)
            %
            % See also DRAW, MBODY\ERASE.
            
            if ~isempty(sen.h)
                try
                    delete(sen.h);
                catch 
                    % ignore errors
                end
                sen.h = [];
            end
        end

        %% TRIGGER
        function [switched num minRange] = trigger(sen, suppress)
            % TRIGGER queries a family of sensors to see if they have been
            % tripped by proximity to one another and updates their trigger
            % state. Syntax:
            %
            %   [switched num minRange] = sensor.trigger(suppress)
            %   [switched num minRange] = trigger(sensor, suppress)
            %
            % switched is true whenever a valid member of the sensor family
            % comes within proximity, measured by sensor.radius, of another
            % valid member of the sensor family. switched is false if the
            % number of sensor family members in proximity decreases or
            % remains the same during the call to sensor.trigger. 
            %
            % num is the number of valid sensor family members within
            % sensor.radius of each other on the call to sensor.trigger.
            %
            % The minRange output contains the minimum range between
            % sensors when trigger was called, minus the sensor's radius
            % value. It is a smooth, more well-behaved function than
            % switched and num for some applications.
            %
            % Validity is determined by the mode of the the sensor family:
            %
            %   - In 'm/s' mode, any MASTER sensor will trigger any SLAVE
            %     sensor. Masters cannot trigger other masters and slaves
            %     cannot trigger other slaves.
            %   - In 'any' mode, ANY sensor will trigger any SLAVE sensor.
            %     Master sensors never trigger each other.
            %
            % Trigger will then call the function listed in the sensor's
            % triggerfn property, which should be a function handle that
            % takes the triggered sensor as an argument.
            %
            % Give a non-false value for 'suppress' if you want the sensor
            % to skip evaluation of the triggerfn.
            %
            % See also MBODY\SOLVE.
            
            %% --Initialize
            if nargin < 2
                suppress = false;
            end
            switched = false;
            minRange = Inf;
            num = 0;
            
            for k = 1:numel(sen.master)
                %% --Check slave sensors against master
                basept = sen.master(k).body.pos + ...
                    ( q2d(sen.master(k).body.att) * sen.master(k).position );
                
                for i = 1:numel(sen.slave)
                    testpt = sen.slave(i).body.pos + ...
                        ( q2d(sen.slave(i).body.att) * sen.slave(i).position );
                    testRange = norm(testpt - basept) - sen.radius;
                    minRange = min([minRange testRange]);
                    if testRange <= 0
                        num = num + 1;
                    end
                end
            end
                
            %% --If in 'any' mode, check slaves against other slaves
            if strcmp( sen.mode, 'any' )
                for j = 1:(numel(sen.slave))
                    
                    basept = sen.slave(j).body.pos + ...
                        ( q2d(sen.slave(j).body.att) * sen.slave(j).position );
                    
                    for i = (j+1):numel(sen.slave)
                        testpt = sen.slave(i).body.pos + ...
                            ( q2d(sen.slave(i).body.att) * sen.slave(i).position );
                        testRange = norm(testpt - basept) - sen.radius;
                        minRange = min([minRange testRange]);
                        if testRange <= 0
                            num = num + 1;
                        end
                    end
                end
            end
            
            %% --Execute triggerfn and return
            if minRange <= 0 % at least one sensor is in proximity
                if num > sen.tripped % a NEW sensor came into proximity
                    switched = true;
                end
                sen.tripped = num;
                if ~suppress
                    sen.triggerfn(sen);
                end
            else
                if sen.tripped > 0 % tripped remains -1 if never triggered
                    sen.tripped = 0;
                end
            end
            
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
% Sensor class
%   Object that triggers simulation termination when in proximity to other
%   sensors, for collision detecting, end-of-simulation detection, etc.

% M-Lint messages to ignore
% #ok<*CTCH>