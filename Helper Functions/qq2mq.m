function [ mq ] = qq2mq(qq )
%qq2mq converts a quirk quaternion to a matlab quaternion
[m,n] = size(qq);
if m == 1
    qq = qq';
end
mq = [qq(4);qq(1:3)]';


end

