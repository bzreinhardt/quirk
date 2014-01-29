classdef magnet < body
    %a magnet is a quirk body with magnetic properties
    %   Detailed explanation goes here
    
    properties
        %position % position of the magnet 
        %dipole   % unit vector of the diple
        m  = 0;    % magnitude of the current in manget
        phase = 0;   % phase of the current in the magnet
        freq  = 0;   % frequency of the current in manget
        
    end
    
    methods
        function obj = magnet(mag, phase, freq, pos, att, varargin)
            if nargin == 0 
                pos = [0;0;0];
                att = [0 0 0 1];
                mag = 0;
                phase = 0;
                freq = 0;
                
            end
            
           %default size of magnet 1cmx1cmx2cm
            obj = obj@body(pos,att,'shape','cylinder');
            set(obj,'sx',0.01);
            set(obj,'sy',0.01);
            set(obj,'sz',0.02);
            obj.m = mag;
            obj.phase = phase;
            obj.freq = freq;
            
                
        end
    end
    
end

