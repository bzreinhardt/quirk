function [x_21,v_21,w_21, att_21] = i2b(body1,body2)
    %finds the position, linear velocity and angular velocity of body 2 in
    %body 1 coordinates
    %assume body1 has no angular velocity TODO change this 
    x_21 = body2.pos - body1.pos;
    v_21 = body2.vel - body1.vel;
    vec2 = att2vec(body2.att,'z');
    rotM = quat2dcm(qq2mq(body1.att));
    rotM2I = inv(quat2dcm(qq2mq(body2.att)));
    att_21 = rotM*vec2;
    w_21 = rotM*rotM2I*body2.om;
   
    att_21(abs(att_21)<1E-10) = 0; %get rid of weird numerical stupid when doing rotations
            
    w_21(abs(w_21)<1E-10) = 0; %get rid of weird numerical stupid when doing rotations
end