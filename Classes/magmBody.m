classdef magmBody < mBody
    %magMbody is a quirk mBody object that keeps track of the magnets in
    %the Mbody
    
    properties
        mags = {};
        plates = {};
    end
    
    methods
        function obj = magmBody(varargin)
            
            obj = obj@mBody(varargin{:});
            
            %add plates and magnets to the system
           for i = 1:numel(varargin)
                switch class(varargin{i})
                    case 'magnet'
                        len = length(obj.mags);
                        obj.mags{len+1} = varargin(i);
                    
                    case 'plate'
                        len = length(obj.plates);
                        obj.plates{len+1} = varargin(i);
                end
           end
           %set up current loops on the plates
           for i = 1:length(obj.plates)
               for j = 1:length(obj.mags);
                   obj.plates{i}{:}.genLoops(obj.mags{j}{:});
               end
           end
            
        end
        
        function obj = add(obj,varargin)
            for i = 1:numel(varargin)
                switch class(varargin{i})
                    case 'magnet'
                        len = length(obj.mags);
                        obj.mags{len+1} = varargin(i);
                        mag = obj.mags{len+1};
                        for j = 1:length(obj.plates)
                            
                            obj.plates{j}{:}.genLoops(mag{:});
                        end
                    case 'plate'
                        len = length(obj.plates);
                        obj.plates{len+1} = varargin(i);
                        plate = obj.plates{len+1};
                        for j = 1:length(obj.mags)
                            plate.genLoops(obj.mags{i}{:});
                        end
                end
            end
            %TODO - make it so whenever you add a plate or a magnet, all
            %the plates and magnets update so that you always have loops
            %living on the plates
                add@mBody(obj,varargin{:});
            
        end
    end
    
end

