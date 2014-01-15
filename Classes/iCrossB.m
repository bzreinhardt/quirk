function [ iXb] = iCrossB(i,x,y,z,magM,magX)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%iXb = zeros(3,1);

    %mF = magFlux(x,y,z,magX,magM);
   %iXb = [0 -i(3) i(2); i(3) 0 -i(1); -i(2) i(1) 0] * magFlux(x,y,z,magX,magM,'all');
    iXb = cross(i,magFlux(x,y,z, magX(:),magM(:),'all'));



end