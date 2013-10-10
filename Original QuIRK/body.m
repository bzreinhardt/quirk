classdef body < hgsetget
    properties
        shape = 'sph';
        sx = 1; sy = 1; sz = 1;
        mass = 1;
        inertia = 2/5*eye(3);

        pos = [0 0 0]';
        att = [0 0 0 1]';
        vel = [0 0 0]';
        om = [0 0 0]';
        time = 0;
        force;
        torque;
        
        color = [0.7 0.7 0.7];
        
        data
    end
    properties (SetAccess = 'private')
        h = [];
        mb = [];
    end %properties

    methods
        
        %% CLEARPARENT
        function bod = clearParent(bod)
            % CLEARPARENT removes the flag that this body is a member of an
            % mBody object. In general, you should not use this command for
            % manual operations. Use MBODY/ADD or MBODY/REMOVE instead. 
            % Syntax:
            %
            %   body = clearParent(body)
            %   body = body.clearParent
            %
            % See MBODY/ADD and MBODY/REMOVE.
            
            warning('BODY:setParent:dontDoThat', ...
                'Manually setting a body''s parent is usually not a good idea. Use MBODY/REMOVE.')
           
            bod.mb = [];
        end
        
        %% CONSTRUCTOR
        function bod = body(pos, att, varargin)
            % BODY creates a body object for multibody simulations.
            % Syntax:
            %
            %   bod = body(position, attitude, 'property', value, ...)
            %
            % Required arguments
            %
            %   position:   initial center-of-mass position of the body
            %   attitude:   initial quaternion rotation from the body to
            %                   the inertial frame
            %
            % Optional properties
            %
            %   inertia dbl     3x3 inertia matrix (may be derived from
            %                       mass and size properties) in body frame
            %   shape   str     [ {sphere} | ellipsoid | ball
            %                       | cuboid | cube | box
            %                       | cylinder ]
            %   size    str/dbl Length, width, and height in 'XxYxZ' form,
            %                       for example, '4x1x2', or [X Y Z] form
            %   mass    dbl     Body mass, in kilograms
            %   color   str/dbl Default color of body for plots
            %   vel     dbl     3x1 initial velocity vector
            %   omega   dbl     3x1 initial angular velocity vector in body
            %                       coordinates
            %   qdot    dbl     4x1 initial quaternion derivative
            %   force   @fn     Force on the body CM as a function of
            %                       position, attitude, velocity, angular
            %                       velocity, and time; in INERTIAL frame
            %   torque  @fn     Torque about the body CM as a function of
            %                       position, attitude, velocity, angular
            %                       velocity, and time; in BODY frame
            %   data    any     User-defined data field
            %   time    dbl     Time, in seconds (currently mostly unused)
            %
            % See also JOINT, FORCE, SENSOR, MBODY.

            %% --Set defaults
            
            % Force/torque defaults
            bod.force = @(pos,q,vel,omega,time) ([0; 0; 0]);
            bod.torque = @(pos,q,vel,omega,time) ([0; 0; 0]);
                
            %% --Parse inputs
            if nargin == 1 && isa(pos, 'body')  % --If given just a body, return that body
                bod = pos;
            elseif nargin > 0   % --If given arguments, construct the specified body
                % Position
                if all(size(pos) == [1 3])
                    pos = pos';
                elseif numel(pos) ~= 3
                    error('BODY:body:input', 'Body position must be a 3-element matrix.')
                end
                bod.pos = pos;

                % Attitude
                if all(size(att) == [1 4])
                    att = att';
                elseif numel(att) ~= 4
                    error('BODY:body:input', 'Body quaternion must be a 4-element matrix.')
                end
                bod.att = att/norm(att);

                % Options
                if numel(varargin)/2 ~= round(numel(varargin)/2)
                    error('BODY:body:input', 'All properties must be paired with values.')
                end
                for i = 1:(length(varargin)/2)
                    if ~ischar(varargin{2*i-1})
                        error('BODY:body:input', 'Property designations must be strings.')
                    end
                    switch lower(varargin{2*i-1})
                        case {'time' 't'}
                            if ~isnumeric(varargin{2*i})
                                error('BODY:body:input', 'Time must be a number.')
                            else
                                bod.time = varargin{2*i};
                            end
                        case {'shape' 'sh'}
                            if ~ischar(varargin{2*i})
                                error('BODY:body:input', 'Shape must be a string.')
                            else
                                switch lower(varargin{2*i})
                                    case {'cylinder' 'cyl'}
                                        bod.shape = 'cyl';
                                    case {'cuboid' 'cube' 'box'}
                                        bod.shape = 'box';
                                    otherwise
                                        bod.shape = 'sph';
                                end
                            end
                        case {'size' 'sz'}
                            if ~ischar(varargin{2*i}) && (~isnumeric(varargin{2*i}) || numel(varargin{2*i}) ~= 3)
                                error('BODY:body:input', 'Size must be a string of the form ''LENGTHxWIDTHxHEIGHT'' or a 3-vector.')
                            else
								if ischar(varargin{2*i})
									k = findstr(lower(varargin{2*i}), 'x');
									bod.sx = str2double(varargin{2*i}(1:(k(1)-1)));
									bod.sy = str2double(varargin{2*i}((k(1)+1):(k(2)-1)));
									bod.sz = str2double(varargin{2*i}((k(2)+1):end));
								else
									bod.sx = varargin{2*i}(1);
									bod.sy = varargin{2*i}(2);
									bod.sz = varargin{2*i}(3);
								end
                            end
                        case {'mass' 'm'}
                            if ~isnumeric(varargin{2*i})
                                error('BODY:body:input', 'Mass must be a number.')
                            else
                                bod.mass = varargin{2*i}(1);
                            end
                        case {'inertia' 'i'}
                            if ~isnumeric(varargin{2*i})
                                error('BODY:body:input', 'Inertia must be numeric.')
                            elseif all(size(varargin{2*i}) ~= [3 3])
                                error('BODY:body:input', 'Inertia dimensions must be 3x3.')
                            else
                                bod.inertia = varargin{2*i};
                            end
                        case {'velocity' 'vel' 'v' 'vel0' 'v0'}
                            if ~isnumeric(varargin{2*i})
                                error('BODY:body:input', 'Velocity must be a number.')
                            else
                                if all(size(varargin{2*i}) == [1 3])
                                    bod.vel = varargin{2*i}';
                                elseif numel(pos) ~= 3
                                    error('BODY:body:input', 'Velocity must be a 3-element matrix.')
                                else
                                    bod.vel = varargin{2*i};
                                end
                            end
                        case {'omega' 'om' 'om0'}
                            if ~isnumeric(varargin{2*i})
                                error('BODY:body:input', 'Angular velocity must be a number.')
                            else
                                if all(size(varargin{2*i}) == [1 3])
                                    bod.om = varargin{2*i}';
                                elseif numel(pos) ~= 3
                                    error('BODY:body:input', 'Angular velocity must be a 3-element matrix.')
                                else
                                    bod.om = varargin{2*i};
                                end
                            end
                        case {'qdot' 'qdot0' 'qd'}
                            if ~isnumeric(varargin{2*i})
                                error('BODY:body:input', 'Quaternion derivative must be a number.')
                            else
                                if all(size(varargin{2*i}) == [1 4])
                                    qd = varargin{2*i}';
                                elseif numel(pos) ~= 4
                                    error('BODY:body:input', 'Quaternion derivative must be a 4-element matrix.')
                                else
                                    qd = varargin{2*i};
                                end
                                
                                bod.om = 2*[-crs(bod.att(1:3))+bod.att(4)*eye(3), -bod.att(1:3)] * qd;
                            end
                        case {'force' 'f'}
                            if ~isa(varargin{2*i}, 'function_handle')
                                error('BODY:body:input', 'Force must be a function handle.')
%                             elseif nargin(varargin{2*i}) ~= 5
%                                 error('BODY:body:input', 'Force must be a function of (position, attitude, velocity, ang vel, and time).')
%                             elseif all(size(varargin{2*i}(zeros(3,1),[0 0 0 1]',zeros(3,1),zeros(3,1),0)) ~= [3 1])
%                                 error('BODY:body:input', 'Force function must return a 3x1 matrix.')
                            else
                                bod.force = varargin{2*i};
                            end
                        case {'torque' 'tor' 'tq'}
                            if ~isa(varargin{2*i}, 'function_handle')
                                error('BODY:body:input', 'Torque must be a function handle.')
%                             elseif nargin(varargin{2*i}) ~= 5
%                                 error('BODY:body:input', 'Torque must be a function of (position, attitude, velocity, ang vel, time).')
%                             elseif all(size(varargin{2*i}(zeros(3,1),[0 0 0 1]',zeros(3,1),zeros(3,1),0)) ~= [3 1])
%                                 error('BODY:body:input', 'Torque function must return a 3x1 matrix.')
                            else
                                bod.torque = varargin{2*i};
                            end
                        case {'color' 'col' 'c'}
                            if ~isnumeric(varargin{2*i}) && ~ischar(varargin{2*i})
                                error('BODY:body:input', 'Color must be a MATLAB colorspec; string or vector.')
                            else
                                bod.color = varargin{2*i};
                            end
                        case {'data' 'dat'}
                            bod.data = varargin{2*i};
                        otherwise
                            warning('BODY:body:input', ...
                                'Discarding unknown property "%s"', varargin{2*i-1})
                    end
                end
                
            else    % --Construct object with default properties

            end

            %% --Calculate inertia if not specified
            if ~any(cellfun(@(c) strncmpi(c, 'i', 1), varargin(1:2:end)))
                switch bod.shape
                    case 'cyl'
                        bod.inertia = bod.mass * diag(...
                            [(3*bod.sy^2+bod.sz^2)/12, ...
                            (3*bod.sx^2+bod.sz^2)/12, ...
                            (bod.sx + bod.sy)^2/8]);
                    case 'box'
                        bod.inertia = bod.mass/12 * diag(...
                            [(bod.sy^2+bod.sz^2), ...
                            (bod.sx^2+bod.sz^2), ...
                            (bod.sx^2+bod.sy^2)]);
                    otherwise %sphere
                        bod.inertia = bod.mass/5 * diag(...
                            [(bod.sy^2+bod.sz^2), ...
                            (bod.sx^2+bod.sz^2), ...
                            (bod.sx^2+bod.sy^2)]);
                end
            end
                
        end
        
        %% DELETE
        function delete(bod)
            % DELETE performs clean-up actions before clearing a body from 
            % memory.
            erase(bod)
            if ~isempty(bod.mb)
                try
                    remove(bod.mb, bod);
                catch
                    % ignore errors
                end
            end
        end
        
        %% DIAMETER
        function dia = diameter(bod)
            % DIAMETER provides a quick estimate of a body's physical size.
            % It returns the maximum of sx, sy, and sz. Syntax:
            %
            %   dia = diameter(bod)
            %   dia = bod.diameter
            %
            % See also RESIZE.
            
            dia = max([bod.sx bod.sy bod.sz]);
        end
        
        %% DISPLAY
        function display(bod)
            % DISPLAY handles the printout of a body object when someone
            % types its name without ending the line in a semicolon.
            
            switch bod.shape
                case 'box'
                    pic = ['     _____   ';
                           '    /\\\\\\  ';
                           '   /  \\\\\\ ';
                           '   \  /####/ ';
                           '    \/####/  '];
                case 'cyl'
                    pic = ['     ___     ';
                           '    (___)    ';
                           '    |  #|    ';
                           '    |  #|    ';
                           '    `---´    '];
                case 'sph'
                    pic = ['             ';
                           '     ,-¸     ';
                           '    (  *)    ';
                           '     `-´     ';
                           '             '];
                otherwise
                    pic = ['             ';
                           '    /---\    ';
                           '    | ? |    ';
                           '    \---/    ';
                           '             '];
            end
            
            disp([inputname(1) ' = '])
            disp(['    ' pic(1,:) ' QuIRK: ' ...
                num2str(bod.sx) ' x ' num2str(bod.sy) ' x ' num2str(bod.sz) ...
                ' m ' bod.shape 'body'])
            disp(['    ' pic(2,:) '  Located at [' num2str(bod.pos(:,end)') '] m'])
            disp(['    ' pic(3,:) '  Attitude   [' num2str(bod.att(:,end)') ']'])
            disp(['    ' pic(4,:) '  Mass        ' num2str(bod.mass) ' kg'])
            disp(['    ' pic(5,:) '  Datatype    ' class(bod.data) ])
            disp(' ');
        end

        %% DRAW
        function h = draw(bod, varargin)
            % DRAW adds a graphic of a body to the current axes. The 
            % graphic will update as this body's properties change. Syntax:
            %
            %   h = draw(body, 'property', value, ...)
            %   h = body.draw('property', value, ...)
            %
            %   body:   body to draw
            %
            % Optional properties
            %
            %   color   dbl/str override color of body
            %   alpha   dbl     Opacity of body, range 0-1
            %   hold    T/F     Draw on top of current graphics if true
            %   snap    T/F     Graphic will not update as body changes if
            %                       this flag is true
            % 
            %   Any surfaceplot property will also work.
            %
            % See also GHOST, ERASE, JOINT\DRAW, MBODY\DRAW, MBODY\GHOST,
            % MBODY\ANIMATE.
            
            %% --Set defaults
            color = bod.color; 
            alph = 1;
            over = false;
            snap = false;
            lit = false;
            
            %% --Parse inputs
            if numel(varargin)/2 ~= round(numel(varargin)/2)
                error('BODY:draw:input', 'All properties must be paired with values.')
            end
            plotargs = true(size(varargin));
            for i = 1:2:(length(varargin))
                if ~ischar(varargin{i})
                    error('BODY:draw:input', 'Property designations must be strings.')
                end
                switch lower(varargin{i})
                    case {'alpha' 'alph' 'a'}
                        if ~isnumeric(varargin{i+1})
                            error('BODY:draw:input', 'Alpha must be numeric.')
                        else
                            alph = varargin{i+1}(1);
                            plotargs(i:(i+1)) = [false false];
                        end
                    case {'color' 'c'}
                        color = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    case {'over' 'hold'}
                        over = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    case 'snap'
                        snap = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                    case 'lit'
                        lit = varargin{i+1}(1);
                        plotargs(i:(i+1)) = [false false];
                end
            end
            
            %% --Create surface points
            switch bod.shape
                case 'cyl'
                    [X,Y,Z] = cylinder([0 ones(1,30) 0.98 ones(1,10) 0], 20);
                    X = bod.sx*X/2; Y = bod.sy*Y/2; Z = bod.sz*(Z-0.5);
                    [X,Y,Z] = rotsurf(X, Y, Z, bod.att);
                    X = X + bod.pos(1); Y = Y + bod.pos(2); Z = Z + bod.pos(3);
                case 'box'
                    [X,Y,Z] = cylinder([0 ones(1,30) 0.98 ones(1,10) 0]/sqrt(2), 4);
                    [X,Y,Z] = rotsurf(X, Y, Z, [0 0 pi/4]);
                    X = bod.sx*X; Y = bod.sy*Y; Z = bod.sz*(Z-0.5);
                    [X,Y,Z] = rotsurf(X, Y, Z, bod.att);
                    X = X + bod.pos(1); Y = Y + bod.pos(2); Z = Z + bod.pos(3);
                otherwise %sphere
                    [X,Y,Z] = ellipsoid(0, 0, 0, bod.sx/2, bod.sy/2, bod.sz/2);
                    [X,Y,Z] = rotsurf(X, Y, Z, bod.att);
                    X = X + bod.pos(1); Y = Y + bod.pos(2); Z = Z + bod.pos(3);
            end
            
            %% --Plot surface
            wasHeld = ishold;
            if over && ~wasHeld
                hold on
            end
            
            h = surf(X, Y, Z, 'FaceColor', color, 'FaceAlpha', alph, ...
                'EdgeAlpha', alph, 'EdgeColor', 'none', varargin{plotargs});
            if ~snap
                bod.h = h;
            end
            
            if ~wasHeld
                hold off
            end
            
            if lit
                light
            end
            
            axis equal
        end
        
        %% ENERGY
        function K = energy(bod)
            % ENERGY returns the current kinetic energy of a body
            % given its current mass properties and velocity. Syntax:
            %
            %   K = energy(body)
            %   K = body.energy
            %
            % See also MOMENTUM, MBODY\ENERGY, MBODY\MOMENTUM.
            
            K = bod.mass/2 * norm(bod.vel)^2 + ...
                bod.om' * (bod.inertia * bod.om)/2;
        end
        
        %% ERASE
        function erase(bod)
            % ERASE clears the graphical representation of a body 
            % from all axes, if one exists. Syntax:
            %
            %   erase(body)
            %   body.erase
            %
            % See also DRAW.
            
            if ~isempty(bod.h)
                try
                    delete(bod.h);
                catch
                    % ignore errors
                end
                bod.h = [];
            end
        end
        
        %% GETFORCE
        function f = getForce(bod)
            % GETFORCE returns the current force vector on a body 
            % if that body has a force function. Syntax:
            %
            %   f = getForce(body)
            %   f = body.getForce
            %
            % This is the same as executing: 
            % f = bod.force(bod.pos, bod.att, bod.time);
            %
            % See also GETTORQUE.
            
            if nargin(bod.force) == 5
                f = bod.force(bod.pos, bod.att, bod.vel, bod.om, bod.time);
            else
                f = bod.force(bod);
            end
        end
        
        %% GETTORQUE
        function T = getTorque(bod)
            % GETTORQUE returns the current torque vector on a body 
            % if that body has a torque function. Syntax:
            %
            %   T = getTorqe(body)
            %   T = body.getTorque
            %
            % This is the same as executing:
            % T = bod.torque(bod.pos, bod.att, bod.time);
            %
            % See also GETFORCE.
            
            if nargin(bod.torque) == 5
                T = bod.torque(bod.pos, bod.att, bod.vel, bod.om, bod.time);
            else
                T = bod.torque(bod);
            end
        end
        
        %% GHOST
        function h = ghost(bod, varargin)
            % GHOST adds a graphic of a body to the current axes 
            % with default opacity of 10%. Refer to DRAW for options.
            %
            % If a body representation already exists, ghost sets the body
            % opacity to 10%.
            %
            %   h = ghost(body, 'property', value, ...)
            %   h = body.ghost('property', value, ...)
            %
            % See also DRAW, MBODY\GHOST.
            
            %% --Call draw()
            if isempty(bod.h)
                bod.h = draw(bod, 'alpha', 0.1, varargin{:});
            else
                set(bod.h, 'FaceAlpha', 0.1);
            end
            
            h = bod.h;
        end
        
        function hilight(bod, color)
            % HILIGHT is a shortcut for body.highlight.
            
            if nargin < 2
                color = 'w';
            end
            bod.highlight(color);
        end
        
        %% HIGHLIGHT
        function highlight(bod, color)
            % HIGHLIGHT paints or unpaints the edges of an already-drawn
            % body for easy identification.
            %
            %   highlight(body, color)
            %   body.highlight(color)
            %
            % Color defaults to white if not present as an argument. You
            % can use HILIGHT as a shorthand.
            %
            % See also DRAW, GHOST, ERASE.
            
            if nargin < 2
                color = 'w';
            end
            
            if ~isempty(bod.h)
                edgeColor = get(bod.h, 'EdgeColor');
                if edgeColor(1) == 'n' % Edges not currently drawn
                    set(bod.h, 'EdgeColor', color);
                else % Edges currently drawn
                    set(bod.h, 'EdgeColor', 'none');
                end
            end
        end
        
        %% MOMENTUM
        function [p H] = momentum(bod)
            % MOMENTUM returns the linear and angular momentum of a body,
            % with its current mass properties and velocity, in the 
            % inertial frame. Syntax:
            %
            %   [p H] = momentum(bod)
            %   [p H] = bod.momentum
            %
            % p:    3x1 linear momentum vector of body
            % H:    3x1 angular momentum vector of body
            %
            % See also ENERGY, MBODY\ENERGY, MBODY\MOMENTUM.
            
            p = bod.mass * bod.vel;
            H = bod.mass * crs(bod.pos) * bod.vel ...
                + q2d(bod.q) * bod.inertia * bod.om;
            
        end
        
        %% Q
        function quat = q(bod)
           % Q is a shorthand to return the body attitude as a quaterion. 
           % Use like bod.att to access the body attitude. Syntax:
           %
           %    quat = bod.q
           %    quat = q(bod)
           
           quat = bod.att;
        end
        
        %% QDOT
        function qd = qdot(bod)
           % QDOT returns the angular velocity of the body as a quaternion
           % derivative. Use like bod.vel to access qdot.
           % Syntax:
           %
           %    qd = bod.qdot
           %    qd = qdot(bod)
           
           qd = 0.5 * [(crs(bod.att(1:3)) + bod.att(4)*eye(3));
               -bod.att(1:3)'] * bod.om;
        end
        
        %% RECOMPUTEINERTIA
        function bod = recomputeInertia(bod)
            % RECOMPUTEINERTIA recalculates the inertia matrix of a body 
            % in the body frame. Call this method if you have manually 
            % changed any of the body size parameters (bod.mass, bod.size, 
            % bod.sx, etc) and want to derive the inertia matrix from the 
            % new parameters. Syntax:
            %
            %   body = recomputeInertia(body)
            %   body = body.recomputeInertia
            %
            % See also RESIZE, MBODY\RECOMPUTEINERTIA.
            switch bod.shape
                case 'cyl'
                    bod.inertia = bod.mass * diag(...
                        [(3*bod.sy^2+bod.sz^2)/12, ...
                        (3*bod.sx^2+bod.sz^2)/12, ...
                        (bod.sx + bod.sy)^2/8]);
                case 'box'
                    bod.inertia = bod.mass/12 * diag(...
                        [(bod.sy^2+bod.sz^2), ...
                        (bod.sx^2+bod.sz^2), ...
                        (bod.sx^2+bod.sy^2)]);
                otherwise %sphere
                    bod.inertia = bod.mass/5 * diag(...
                        [(bod.sy^2+bod.sz^2), ...
                        (bod.sx^2+bod.sz^2), ...
                        (bod.sx^2+bod.sy^2)]);
            end
        end
        
        %% RESIZE
        function bod = resize(bod, size)
            % RESIZE provides a convenient shorthand way to specify the  
            % spatial extent of a body.
            %
            %   body = resize(body, newSize)
            %   body = body.resize(newSize)
            %
            %   newSize:    String in 'LENGTHxWIDTHxHEIGHT' format; for
            %                   example, '3x1x2' changes the body to length
            %                   3 m along the x axis, 1 m along the y axis,
            %                   and 2 m along the z axis. newSize can also
			%					be a 3-element vector.
            %
            % See also RECOMPUTEINERTIA, DIAMETER.
            
            %% --Parse inputs
            if ~ischar(size) && (~isnumeric(size) || numel(size) ~= 3)
                error('BODY:size:input', ...
                    'Size must be a string of the form ''LENGTHxWIDTHxHEIGHT'' or a 3-vector.')
            else
				if ischar(size)
					k = findstr(size, 'x');
					newSX = str2double(size(1:(k(1)-1)));
					newSY = str2double(size((k(1)+1):(k(2)-1)));
					newSZ = str2double(size((k(2)+1):end));
				else
					newSX = size(1);
					newSY = size(2);
					newSZ = size(3);
				end
            end
            
            %% --Move representation
            if ~isempty(bod.h)
                try
                    [X Y Z] = rotsurf(get(bod.h, 'XData') - bod.pos(1), ...
                        get(bod.h, 'YData') - bod.pos(2), ...
                        get(bod.h, 'ZData') - bod.pos(3), ...
                        [bod.att(1:3); -bod.att(4)]);
                    [X Y Z] = rotsurf(newSX/bod.sx * X + bod.pos(1), ...
                        newSY/bod.sy * Y + bod.pos(2), ...
                        newSZ/bod.sz * Z + bod.pos(3), ...
                        bod.att);
                    set(bod.h, 'XData', X , 'YData', Y, 'ZData', Z);
                    axis equal
                catch 
                    bod.h = [];
                end
            end
            
            %% --Store new value
            bod.sx = newSX;
            bod.sy = newSY;
            bod.sz = newSZ;
            
            if ~isempty(bod.mb)
                warning('off', 'MBODY:unSolve:dontDoThat');
                bod.mb.unSolve;
                warning('on', 'MBODY:unSolve:dontDoThat');
            end
        end
        
        %% SET.ATT
        function bod = set.att(bod, newAtt)
            
            %% --Parse inputs
            if ~isnumeric(newAtt)
                error('BODY:set:input', 'Body attitude must be numeric.')
            elseif all(size(newAtt) == [1 4])
                newAtt = newAtt';
            elseif numel(newAtt) ~= 4
                error('BODY:set:input', 'Body attitude must be a 4-element matrix.')
            end
            
            newAtt = newAtt/norm(newAtt);
            
            %% --Move representation
            if ~isempty(bod.h)
                try
                    dq = q2d(newAtt)*q2d(bod.att)';
                    [X Y Z] = rotsurf(get(bod.h, 'XData') - bod.pos(1), ...
                        get(bod.h, 'YData') - bod.pos(2), ...
                        get(bod.h, 'ZData') - bod.pos(3), ...
                        dq);

                    set(bod.h, 'XData', X + bod.pos(1), ...
                        'YData', Y + bod.pos(2), 'ZData', Z + bod.pos(3));
                    axis equal
                catch
                    bod.h = [];
                end
            end
            
            %% --Store new value
            bod.att = newAtt;
        end
        
        %% SET.COLOR
        function bod = set.color(bod, newColor)
            bod.color = newColor;
            if ~isempty(bod.h)
                try
                    set(bod.h, 'FaceColor', newColor);
                catch
                    bod.h = [];
                end
            end
        end
        
        %% SET.FORCE
        function bod = set.force(bod, newForce)
            bod.force = newForce;
            
            if ~isempty(bod.mb)
                warning('off', 'MBODY:unSolve:dontDoThat');
                bod.mb.unSolve;
                warning('on', 'MBODY:unSolve:dontDoThat');
            end
        end
        
        %% SET.INERTIA
        function bod = set.inertia(bod, newInertia)
            bod.inertia = newInertia;
            
            if ~isempty(bod.mb)
                warning('off', 'MBODY:unSolve:dontDoThat');
                bod.mb.unSolve;
                warning('on', 'MBODY:unSolve:dontDoThat');
            end
        end
        
        %% SET.MASS
        function bod = set.mass(bod, newMass)
            bod.mass = newMass;
            
            if ~isempty(bod.mb)
                warning('off', 'MBODY:unSolve:dontDoThat');
                bod.mb.unSolve;
                warning('on', 'MBODY:unSolve:dontDoThat');
            end
        end
        
        %% SET.OM
        function bod = set.om(bod, newOm)
            
            bod.om = newOm;
        end
        
        %% SET.POS
        function bod = set.pos(bod, newPos)
            
            %% --Parse inputs
            if ~isnumeric(newPos)
                error('BODY:set:input', 'Body position must be numeric.')
            elseif all(size(newPos) == [1 3])
                newPos = newPos';
            elseif numel(newPos) ~= 3
                error('BODY:set:input', 'Body position must be a 3-element matrix.')
            end
            
            %% --Move representation
            if ~isempty(bod.h)
                try
                    set(bod.h, 'XData', get(bod.h, 'XData') + newPos(1)-bod.pos(1), ...
                        'YData', get(bod.h, 'YData') + newPos(2)-bod.pos(2), ...
                        'ZData', get(bod.h, 'ZData') + newPos(3)-bod.pos(3));
                    axis equal
                catch
                    bod.h = [];
                end
            end
            
            %% --Store new value
            bod.pos = newPos;
        end
        
        %% SET.SHAPE
        function bod = set.shape(bod, newShape)
            %% --Parse inputs
            if ~ischar(newShape)
                error('BODY:set:input', 'Shape must be a string.')
            end
            
            %% --Make change
            switch lower(newShape)
                case {'cylinder' 'cyl'}
                    bod.shape = 'cyl';
                case {'cuboid' 'cube' 'box'}
                    bod.shape = 'box';
                otherwise
                    bod.shape = 'sph';
            end

            %% --Redraw
            if ~isempty(bod.h)
                try
                    alph = get(bod.h, 'FaceAlpha');
                    delete(bod.h);
                    draw(bod, 'alpha', alph);
                catch
                    bod.h = [];
                end
            end
        end
        
        %% SET.SX
        function bod = set.sx(bod, newSX)
            %% --Parse inputs
            if ~isnumeric(newSX)
                error('BODY:set:input', 'Body length must be numeric.')
            elseif numel(newSX) ~= 1
                error('BODY:set:input', 'Body length must be scalar.')
            end
            
            %% --Move representation
            if ~isempty(bod.h)
                try
                    [X Y Z] = rotsurf(get(bod.h, 'XData') - bod.pos(1), ...
                        get(bod.h, 'YData') - bod.pos(2), ...
                        get(bod.h, 'ZData') - bod.pos(3), ...
                        [bod.att(1:3); -bod.att(4)]);
                    [X Y Z] = rotsurf(newSX/bod.sx * X + bod.pos(1), ...
                        Y + bod.pos(2), ...
                        Z + bod.pos(3), ...
                        bod.att);
                    set(bod.h, 'XData', X , 'YData', Y, 'ZData', Z);
                    axis equal
                catch
                    bod.h = [];
                end
            end
            
            %% --Store new value
            bod.sx = newSX;
            
            if ~isempty(bod.mb)
                warning('off', 'MBODY:unSolve:dontDoThat');
                bod.mb.unSolve;
                warning('on', 'MBODY:unSolve:dontDoThat');
            end
        end
        
        %% SET.SY
        function bod = set.sy(bod, newSY)
            %% --Parse inputs
            if ~isnumeric(newSY)
                error('BODY:set:input', 'Body width must be numeric.')
            elseif numel(newSY) ~= 1
                error('BODY:set:input', 'Body width must be scalar.')
            end
            
            %% --Move representation
            if ~isempty(bod.h)
                try
                    [X Y Z] = rotsurf(get(bod.h, 'XData') - bod.pos(1), ...
                        get(bod.h, 'YData') - bod.pos(2), ...
                        get(bod.h, 'ZData') - bod.pos(3), ...
                        [bod.att(1:3); -bod.att(4)]);
                    [X Y Z] = rotsurf(X + bod.pos(1), ...
                        newSY/bod.sy * Y + bod.pos(2), ...
                        Z + bod.pos(3), ...
                        bod.att);
                    set(bod.h, 'XData', X , 'YData', Y, 'ZData', Z);
                    axis equal
                catch
                    bod.h = [];
                end
            end
            
            %% --Store new value
            bod.sy = newSY;
            
            if ~isempty(bod.mb)
                warning('off', 'MBODY:unSolve:dontDoThat');
                bod.mb.unSolve;
                warning('on', 'MBODY:unSolve:dontDoThat');
            end
        end
        
        %% SET.SZ
        function bod = set.sz(bod, newSZ)
            %% --Parse inputs
            if ~isnumeric(newSZ)
                error('BODY:set:input', 'Body height must be numeric.')
            elseif numel(newSZ) ~= 1
                error('BODY:set:input', 'Body height must be scalar.')
            end
            
            %% --Move representation
            if ~isempty(bod.h)
                try
                    [X Y Z] = rotsurf(get(bod.h, 'XData') - bod.pos(1), ...
                        get(bod.h, 'YData') - bod.pos(2), ...
                        get(bod.h, 'ZData') - bod.pos(3), ...
                        [bod.att(1:3); -bod.att(4)]);
                    [X Y Z] = rotsurf(X + bod.pos(1), ...
                        Y + bod.pos(2), ...
                        newSZ/bod.sz * Z + bod.pos(3), ...
                        bod.att);
                    set(bod.h, 'XData', X , 'YData', Y, 'ZData', Z);
                    axis equal
                catch
                    bod.h = [];
                end
            end
            
            %% --Store new value
            bod.sz = newSZ;
            
            if ~isempty(bod.mb)
                warning('off', 'MBODY:unSolve:dontDoThat');
                bod.mb.unSolve;
                warning('on', 'MBODY:unSolve:dontDoThat');
            end
        end
        
        %% SET.TORQUE
        function bod = set.torque(bod, newTorque)
            bod.torque = newTorque;
            
            if ~isempty(bod.mb)
                warning('off', 'MBODY:unSolve:dontDoThat');
                bod.mb.unSolve;
                warning('on', 'MBODY:unSolve:dontDoThat');
            end
        end
        
        %% SET.VEL
        function bod = set.vel(bod, newVel)
            
            bod.vel = newVel;
        end
        
        %% SETPARENT
        function bod = setParent(bod, mBod)
            % SETPARENT flags this body as a member of an mBody object. In
            % general, you should not use this command for manual
            % operations. Use MBODY/ADD or MBODY/REMOVE instead. Syntax:
            %
            %   body = setParent(body, mBody)
            %   body = body.setParent(mBody)
            %
            % See MBODY/ADD, MBODY/REMOVE.
            
            warning('BODY:setParent:dontDoThat', ...
                'Manually setting a body''s parent is usually not a good idea.\nUse MBODY/ADD and MBODY/REMOVE.')
            
            if ~isempty(bod.mb)
                warning('BODY:setParent:overwrite', ...
                    'This body is already a member of an mBody.\nChanging the parent may cause problems. Use MBODY/ADD and MBODY/REMOVE.')
            end
            
            if ~isa(mBod, 'mBody')
                error('BODY:setParent:input', ...
                    'You must set the parent to an mBody.')
            end
            
            bod.mb = mBod;
            
            warning('off', 'MBODY:unSolve:dontDoThat');
            mBod.unSolve;
            warning('on', 'MBODY:unSolve:dontDoThat');
        end
        
        function mbod = solve(bod, varargin)
            % SOLVE, called on a body object, returns a solved mBody that
            % consists only of that body, with whatever force and torque
            % functions it includes. See MBODY/SOLVE for full syntax and
            % options.
            
            mbod = mBody(bod);
            mbod = mbod.solve(varargin{:});
        end
        
    end %methods

%     events
% 
%     end %events
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
% Body class
%   Object describing a rigid body and encapsulating its current state.



% M-Lint messages to ignore
%#ok<*CTCH>
%#ok<*PROP>