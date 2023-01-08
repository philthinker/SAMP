function [SE3] = qp2SE3(qp, q, p)
%qp2SE3 Transform quaternion and position to the SE(3) form.
%   - MATLAB Robotic System Toolbox is required.
%   qp: N x 7 or [], [qw qx qy qz x y z] data.
%   q: N x 4 or [], [qw qx qy qz], quat data.
%   p: N x 3 or [], [x y z], position data.
%   --------------------------------------------------
%   SE3: 4 x 4 x N, the SE(3) data.

if nargin < 3
    p = [];
end
if nargin < 2
    q = [];
end

%% Input detect
N = max([size(qp,1), size(q,1), size(p,1)]);
if N <= 0
    SE3 = [];
    return;
end

if ~isempty(qp)
    % Ignore the argument q and p.
    q = qp(:,1:4);
    p = qp(:,5:7);
else
    if isempty(q)
        q = repmat([1 0 0 0],[N,1]);
    end
    if isempty(p)
        p = repmat([0 0 0], [N,1]);
    end
end

%% Transform
SE3 = repmat(eye(4),[1,1,N]);

SE3(1:3,4,:) = permute(p,[2, 3, 1]);
SO3 = quat2rotm(q);
SE3(1:3,1:3,:) = SO3;

end

