function [DataOut] = SE3toPQ(SE3In, quatLeftFlag)
%SE3toPQ Transform SE(3) data to position-quaternion form
%   Robotics systems toolbox is required
%   SE3In: 4 x 4 x N, data in
%   quatLeftFlag: Boolean, true for output [ qw qx qy qz ... ], false for [
%   x y z ... ]. (Default: false)
%   -------------------------------------------------
%   DataOut: N x 7, [x y z qw, qx, qy, qz] or [ qw qy qz x y z ]
if nargin < 2
    quatLeftFlag = false;
end
N = size(SE3In,3);
DataOut = zeros(N,7);
if quatLeftFlag
    % [ qw qx qy qz x y z ]
    DataOut(:,5:7) = permute( SE3In(1:3,4,:), [3,1,2] );
    DataOut(:,1:4) = tform2quat(SE3In);
else
    % [ x y z qw qx qy qz ]
    DataOut(:,1:3) = permute( SE3In(1:3,4,:), [3,1,2] );
    DataOut(:,4:7) = tform2quat(SE3In);
end

end

