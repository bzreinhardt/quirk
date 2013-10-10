% write a vector w in cross-product-matrix form.
%            wcrs=[ 0    -w(3)  w(2)
%                   w(3)  0    -w(1)
%                  -w(2)  w(1)  0]; 

% Originally written by Mason Peck, Cornell University
% Included as a supporting function for QuIRK


function [wcrs] = crs(w)
           wcrs=[ 0    -w(3)  w(2)
                  w(3)  0    -w(1)
                 -w(2)  w(1)  0];   
