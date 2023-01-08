function [invSE3Data] = fastInvSE3(SE3Data)
%fastInvSE3 Calculate the inverse of SE(3) data.
%   SE3Data: 4 x 4 x N, SE(3) data.
%   -------------------------------------------------
%   invSE3Data: 4 x 4 x N, SE(3) data.

R1 = SE3Data(1:3,1:3,1);
p1 = SE3Data(1:3,4,1);
invSE3Data = eye(4);
invSE3Data(1:3,1:3) = R1';
invSE3Data(1:3,4) = -(R1')*p1;
if size(SE3Data,3) > 1
    N = size(SE3Data,3);
    invSE3Data = repmat(invSE3Data,[1,1,N]);
    for i =2:N
        R = SE3Data(1:3,1:3,i);
        p = SE3Data(1:3,4,i);
        invSE3Data(1:3,1:3,i) = R';
        invSE3Data(1:3,4,i) = -(R')*p;
    end
end

end

