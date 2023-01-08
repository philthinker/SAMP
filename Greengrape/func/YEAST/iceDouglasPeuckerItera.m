function [KeyPoint,Trajs,ID] = iceDouglasPeuckerItera(Traj,THD)
%iceDouglasPeuckerItera One iteration of the Douglas-Peucker algorithm.
%   - Note that the trajectory must not be closed!
%   - N must not be less than 3!
%   Traj: D x N, the raw trajectory data.
%   THD: Scalar >0, the threshold.
%   -------------------------------------------------
%   KeyPoint: D x 1 or [], the key point.
%   Trajs: 1 x 2 or 1 x 1 cell, the trajectory segments.
%   ID: Integer or [], the ID of the key point in the given trajectory.

%% Too few data points.

KeyPoint = [];
Trajs = [];
ID = [];

if size(Traj,2) < 3
    return;
end

%% Estimate the key point.

p0 = Traj(:,1);
p1 = Traj(:,end);

v = p1 - p0; v = v/norm(v);
d = zeros(1,size(Traj,2));
for i = 2:size(Traj,2)-1
    pt = Traj(:,i);
    d(i) = norm( pt - (p0 + ((pt-p0)'*v)*v) );
end

if max(d) >= THD
    [~, ID] = max(d);
    KeyPoint = Traj(:,ID);
    Trajs = cell(1,2);
    Trajs{1} = Traj(:,1:ID);
    Trajs{2} = Traj(:,ID:end);
end


