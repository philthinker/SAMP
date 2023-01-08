function [traj] = trajGen_GMR(obj,s,h)
%trajGen_GMR Trajectory generation via GMR.
%   s: 1 x N, the state sequence
%   h: K x N, the likelihood of each state in the sequence
%   -----------------------------------------
%   traj: D x N, the motion trajectory
%   @BlackTea1
N  = length(s);
traj = zeros(obj.D,N);
for i = 1:N
    tmpP = zeros(obj.D,1);
    for k = 1:obj.K
        tmpP  = tmpP + h(k,i)*obj.Mu(:,k);
    end
    traj(:,i) = tmpP;
end
end