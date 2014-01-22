function [qq ] = mq2qq(mq)
%converts a quirk quaternion to a matlab quaternion
qq = [mq(2:4),mq(1)];

end

